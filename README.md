# erp42_costmap_maker
erp42_costmap_maker

## Requirements
```bash
$ export PYTHONPATH=
$ python3 -m venv .venv
$ source .venv/bin/activate

$ pip install --upgrade pip
$ pip install -r requirements.txt
$ deactivate
```

## Usage
- If you just want to show map preview, set `debug_mode` (line21)  as `True`. Then code don't calculate costmap.
- Convert waypoint to costmap
```bash
$ source .venv/bin/activate
$ python3 scripts/erp42_costmap_maker.py
$ deactivate
```
- Change some yaml parts after run
  - change `'[-14.0, -51.0, 0] please remove single quote'` to `[-14.0, -51.0, 0]`
  - change `negate` from 0 to 1 or from 1 to 0

```yaml
free_thresh: 0.196
image: ./map_songdo_221017_v1.pgm
negate: 0
occupied_thresh: 0.65
origin: '[-14.0, -51.0, 0] please remove single quote'
resolution: 0.1
```

