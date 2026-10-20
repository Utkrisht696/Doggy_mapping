"""Read-only original recording -> passive sensor inputs for the live launch."""
from pathlib import Path
import sys,json
from collections import Counter
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'local/map_decoder'))
from mcap.reader import make_reader
import rosbag2_py

HERE=Path(__file__).resolve().parent
source=Path('/home/jain/go2_bags/robot_20261018_112353')
out=HERE/'sensor_inputs'
assert not out.exists()
topics={'/utlidar/cloud':'sensor_msgs/msg/PointCloud2','/sportmodestate':'unitree_go/msg/SportModeState',
        '/utlidar/robot_odom':'nav_msgs/msg/Odometry','/lf/lowstate':'unitree_go/msg/LowState'}
files=sorted(source.glob('*.mcap'));before={str(f):(f.stat().st_size,f.stat().st_mtime_ns) for f in files}
writer=rosbag2_py.SequentialWriter();writer.open(rosbag2_py.StorageOptions(uri=str(out),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
for name,typ in topics.items():writer.create_topic(rosbag2_py.TopicMetadata(name=name,type=typ,serialization_format='cdr'))
counts=Counter()
for file in files:
    with file.open('rb') as stream:
        for schema,channel,message in make_reader(stream).iter_messages(topics=list(topics)):
            assert schema.name==topics[channel.topic] and channel.message_encoding=='cdr'
            writer.write(channel.topic,message.data,message.log_time);counts[channel.topic]+=1
del writer
assert before=={str(f):(f.stat().st_size,f.stat().st_mtime_ns) for f in files}
old=json.loads((ROOT/'reports/bag_comparisons/return_drift_20261018_112353/input_audit.json').read_text())
assert all(counts[k]==old['counts'][k] for k in topics)
report=dict(source=str(source),counts=dict(counts),original_size_and_mtime_unchanged=True,
            source_and_receipt_timestamps_preserved=True,robot_commands_included=False)
(HERE/'input_audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)
