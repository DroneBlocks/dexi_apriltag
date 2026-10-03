#!/usr/bin/env python3
"""Import docs/node-red-tag-navigation-flow.json into a Node-RED instance.

    python3 tools/deploy_node_red_flow.py http://<aircraft>:1880

Replaces the 'DEXI Tag Navigation' tab if present, keeps every other flow, and
points the imported nodes at the ros2-websocket-server node the target already
has (ids differ between images). Node-RED's admin API must be open (DEXI default).
"""
import json, pathlib, sys, urllib.request

TAB = 'tagnav_tab'
base = (sys.argv[1] if len(sys.argv) > 1 else 'http://127.0.0.1:1880').rstrip('/')
flow_path = pathlib.Path(__file__).resolve().parent.parent / 'docs' / 'node-red-tag-navigation-flow.json'
nodes = json.loads(flow_path.read_text())

current = json.load(urllib.request.urlopen(base + '/flows'))
keep = [n for n in current if n.get('id') != TAB and n.get('z') != TAB]
ours = {n.get('server') for n in nodes if n.get('server')}
servers = [n['id'] for n in keep if n.get('type') == 'ros2-websocket-server']
if servers and not ours <= set(servers):
    for n in nodes:
        if n.get('server'):
            n['server'] = servers[0]
    print('rosbridge server node:', servers[0])
elif not servers:
    sid = next(iter(ours))
    keep.append({'id': sid, 'type': 'ros2-websocket-server', 'name': 'rosbridge', 'url': '${ROS2_WEBSOCKET_URL}'})
    print('target had no ros2-websocket-server node; added one on $ROS2_WEBSOCKET_URL')

# node-red-dashboard allows exactly one ui_base; keep the target's if it has one.
if any(n.get('type') == 'ui_base' for n in keep):
    nodes = [n for n in nodes if n.get('type') != 'ui_base']
    print('target already has a ui_base; keeping it')

req = urllib.request.Request(base + '/flows', data=json.dumps(keep + nodes).encode(), method='POST',
                             headers={'Content-Type': 'application/json', 'Node-RED-Deployment-Type': 'full'})
print('deploy', base, urllib.request.urlopen(req).status, '— open', base + '/#flow/' + TAB)
