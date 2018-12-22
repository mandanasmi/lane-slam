from collections import namedtuple

from .exception_utils import raise_wrapped
from .exceptions import DTConfigException
from .friendly_path_imp import friendly_path
from .text_utils import indent


Person = namedtuple('Person', 'name email')
PackageXML = namedtuple('PackageXML', 
                        ['name', 'version', 
                         'maintainers', 'authors', 
                         'license', 'description'])

def read_package_xml_info(filename):
    import xml.etree.ElementTree as ET
    try:
        tree = ET.parse(filename)
        root = tree.getroot()
    except Exception as e:
        msg = 'Could not read "package.xml":\n%s' % indent(str(e), '  ')
        raise DTConfigException(msg) 
    
    try:
        
        version, _attrs = get_tag_and_attributes(root, 'version', default=None)
        name, _attrs = get_tag_and_attributes(root, 'name', default=None)
        license_, _attrs = get_tag_and_attributes(root, 'license', default=None)
        description, _attrs = get_tag_and_attributes(root, 'description', default=None)
        maintainers = get_maintainers(root)
        authors = get_authors(root)
        
        return PackageXML(name=name, version=version, maintainers=maintainers, authors=authors,
                          license=license_, description=description)
    except DTConfigException as e:
        msg = 'Could not read info from %s' % friendly_path(filename)
        raise_wrapped(DTConfigException, e, msg)

def get_person(element):
    email = element.attrib.get('email', None)
    name = element.text
    p = Person(name=name, email=email) 
    return p

def get_maintainers(root):
    res = []
    for e in root:
        if e.tag == 'maintainer':
            res.append(get_person(e))
    return res

def get_authors(root):
    res = []
    for e in root:
        if e.tag == 'author':
            res.append(get_person(e))
    return res
    
def get_tag_and_attributes(root, name, default=-1):
    for e in root:
        if e.tag == name:
            return e.text, e.attrib
    if default != -1:
        return default
    raise KeyError(name)

