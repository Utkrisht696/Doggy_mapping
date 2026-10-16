#!/usr/bin/env python3
"""Passive DDS/ROS discovery and bounded telemetry sampling. No command publishers."""
import argparse
import csv
import json
import math
import os
import numbers
import struct
import time
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def compact(value, depth=0):
    if depth > 6:
        return "<nested>"
    if hasattr(value, "get_fields_and_field_types"):
        return {k: compact(getattr(value, k), depth + 1)
                for k in value.get_fields_and_field_types()}
    if isinstance(value, str):
        return value[:2500]
    if isinstance(value, bool) or value is None:
        return value
    if isinstance(value, numbers.Integral):
        return int(value)
    if isinstance(value, numbers.Real):
        return float(value) if math.isfinite(value) else str(value)
    try:
        n = len(value)
        if n > 40:
            return {"length": n, "first_12": [compact(x, depth + 1) for x in value[:12]]}
        return [compact(x, depth + 1) for x in value]
    except TypeError:
        return str(value)


def summarize(msg):
    if hasattr(msg, "point_step") and hasattr(msg, "data"):
        out = {k: compact(getattr(msg, k)) for k in (
            "header", "height", "width", "fields", "is_bigendian", "point_step", "row_step", "is_dense")}
        out["data_bytes"] = len(msg.data)
        fields = {f.name: f for f in msg.fields}
        xyz = [fields.get(k) for k in ("x", "y", "z")]
        if all(f is not None and f.datatype == 7 for f in xyz) and msg.point_step:
            total = msg.height * msg.width
            stride = max(1, math.ceil(total / 2000))
            points = []
            invalid = 0
            zero = 0
            endian = ">" if msg.is_bigendian else "<"
            for index in range(0, total, stride):
                row, col = divmod(index, msg.width)
                offset = row * msg.row_step + col * msg.point_step
                p = [struct.unpack_from(endian + "f", msg.data, offset + f.offset)[0] for f in xyz]
                if all(math.isfinite(v) for v in p):
                    points.append(p)
                    zero += int(all(v == 0 for v in p))
                else:
                    invalid += 1
            out["sampled_points"] = len(points) + invalid
            out["sampled_nonfinite"] = invalid
            out["sampled_zero_xyz"] = zero
            if points:
                out["sample_xyz_bounds_m"] = [[min(p[i] for p in points), max(p[i] for p in points)] for i in range(3)]
                out["sample_xyz_first_3"] = points[:3]
        return out
    return compact(msg)


def endpoint(info):
    q = info.qos_profile
    return {"node_name": info.node_name, "node_namespace": info.node_namespace,
            "topic_type": info.topic_type, "gid": list(info.endpoint_gid),
            "qos": {"reliability": q.reliability.name, "durability": q.durability.name,
                    "history": q.history.name, "depth": q.depth}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=45)
    parser.add_argument("--discovery", type=float, default=8)
    parser.add_argument("--output", default="reports/live_scan")
    parser.add_argument("--only", action="append", help="Subscribe only to these exact topics (repeatable); still inventory the whole graph")
    args = parser.parse_args()
    if args.duration <= 0 or args.discovery < 0:
        parser.error("duration must be positive and discovery nonnegative")
    rclpy.init()
    node = Node("doggy_passive_topic_scan", enable_rosout=False, start_parameter_services=False)
    end = time.monotonic() + args.discovery
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    topics = sorted(node.get_topic_names_and_types())
    records = {}
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    # Best effort/volatile accepts both reliable and best-effort live publishers.
    # Historical transient-local samples are deliberately not requested.
    for name, types in topics:
        item = {"types": types, "publishers": [endpoint(x) for x in node.get_publishers_info_by_topic(name)],
                "subscribers_before_scan": [endpoint(x) for x in node.get_subscriptions_info_by_topic(name)],
                "received": 0, "bytes": 0, "samples": [], "errors": []}
        records[name] = item
        item["sampling_requested"] = not args.only or name in args.only
        if not item["sampling_requested"]:
            continue
        if len(types) != 1:
            item["errors"].append("Multiple types: no subscription created")
            continue
        try:
            cls = get_message(types[0])
        except Exception as exc:
            item["errors"].append("Message definition unavailable: " + str(exc))
            continue

        def callback(raw, item=item, cls=cls):
            now = time.monotonic()
            item.setdefault("first_receipt_monotonic", now)
            item["last_receipt_monotonic"] = now
            item["received"] += 1
            item["bytes"] += len(raw)
            # Three samples spaced through the window; avoid deserializing every cloud.
            if len(item["samples"]) < 3 and now >= item.get("next_sample", 0):
                item["next_sample"] = now + args.duration / 3
                try:
                    msg = deserialize_message(raw, cls)
                    sample = {"receipt_unix": time.time(), "value": summarize(msg)}
                    stamp = getattr(getattr(msg, "header", None), "stamp", None)
                    if stamp is not None:
                        stamp_s = stamp.sec + stamp.nanosec / 1e9
                        sample["header_stamp_unix"] = stamp_s
                        sample["host_minus_header_stamp_s"] = sample["receipt_unix"] - stamp_s
                    item["samples"].append(sample)
                except Exception as exc:
                    item["errors"].append("Sample decode failed: " + str(exc))
        try:
            node.create_subscription(cls, name, callback, qos, raw=True)
        except Exception as exc:
            item["errors"].append("Subscription failed: " + str(exc))

    started_local = datetime.now(ZoneInfo("Australia/Melbourne")).isoformat()
    start = time.monotonic()
    print(f"Sampling {len(topics)} discovered topics for {args.duration:g}s", flush=True)
    while time.monotonic() - start < args.duration:
        rclpy.spin_once(node, timeout_sec=0.02)
    elapsed = time.monotonic() - start
    for name, item in records.items():
        count = item["received"]
        span = item.get("last_receipt_monotonic", 0) - item.get("first_receipt_monotonic", 0)
        item["observed_rate_hz"] = round((count - 1) / span, 3) if count > 1 and span else 0
        item["window_rate_hz"] = round(count / elapsed, 3)
        item["received_mbps"] = round(item["bytes"] * 8 / elapsed / 1e6, 3)
        item["publishers_after_scan"] = [endpoint(x) for x in node.get_publishers_info_by_topic(name)]
        item.pop("next_sample", None)
    expected = ["/tf", "/tf_static", "/map", "/scan", "/odom", "/cmd_vel", "/joint_states",
                "/battery_state", "/navigate_to_pose/_action/status", "/follow_waypoints/_action/status"]
    result = {"started_local": started_local, "elapsed_s": elapsed,
              "robot_model_user_reported": "Go2 Pro X", "interface": "enp7s0",
              "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
              "rmw": os.environ.get("RMW_IMPLEMENTATION", "default (Fast DDS on Humble)"),
              "method": "Read-only subscriptions; best-effort volatile; no command publishers or service calls. Rates are laptop receipt rates, not guaranteed source rates.",
              "nodes": node.get_node_names_and_namespaces(),
              "services": node.get_service_names_and_types(),
              "expected_nav2_topics_present": {x: x in dict(node.get_topic_names_and_types()) for x in expected},
              "topics": records}
    target = Path(args.output)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.with_suffix(".json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    with target.with_suffix(".csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["topic", "type", "publishers", "subscribers_before_scan", "messages", "observed_hz", "Mbps", "frames", "errors"])
        for name, item in records.items():
            frames = sorted({s["value"].get("header", {}).get("frame_id", "") for s in item["samples"] if isinstance(s["value"], dict)})
            writer.writerow([name, ";".join(item["types"]), len(item["publishers"]), len(item["subscribers_before_scan"]),
                             item["received"], item["observed_rate_hz"], item["received_mbps"], ";".join(frames), ";".join(item["errors"])])
    node.destroy_node()
    rclpy.shutdown()
    active = sum(x["received"] > 0 for x in records.values())
    print(f"Saved {target}.json and .csv: {len(records)} topics, {active} received data", flush=True)


if __name__ == "__main__":
    main()
