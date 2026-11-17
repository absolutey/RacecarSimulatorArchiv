#!/usr/bin/env python3
"""Extract each distinct failure from the ROS colcon XML files without guessing a cause."""
from pathlib import Path
import xml.etree.ElementTree as ET
import sys
root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path.cwd()
base = root/'build'
if not base.exists():
    raise SystemExit('BUILD_DIR_NOT_FOUND='+str(base))
files=sorted(set(base.glob('*/test_results/**/*.xml')) | set(base.glob('*/pytest.xml')))
failed=[]
for f in files:
    try: r=ET.parse(f).getroot()
    except (ET.ParseError, OSError): continue
    for c in r.iter('testcase'):
        for e in list(c):
            if e.tag not in ('failure', 'error'): continue
            failed.append((str(f.relative_to(root)),c.attrib.get('classname',''),c.attrib.get('name',''),e.tag,(e.attrib.get('message','') or (e.text or '')).strip()[:700]))
for path,cls,name,kind,msg in failed:
    print(f'FAIL_FILE={path} | CLASS={cls} | TEST={name} | KIND={kind} | DETAIL={msg}')
print('XML_FAILURE_RECORDS=',len(failed),' XML_FILES=',len(files))
if not files:
    raise SystemExit('NO_XML_RESULTS_FOUND')
