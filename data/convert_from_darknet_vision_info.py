import sys
import argparse
import xml.etree.ElementTree as et
 

def main():
    argv = sys.argv
    if len(argv) != 2:
        return
    meta_data = open(argv[1]+".names", "r")
    id = 0
    root = et.Element('vision_info')
    for line in meta_data:
        name = line.replace('\n','')
        p = et.SubElement(root,'class',{'name':name,"id":str(id)})
        id = id + 1
    tree = et.ElementTree(element=root)
    tree.write(argv[1]+".xml")

if __name__ == '__main__':
    main()