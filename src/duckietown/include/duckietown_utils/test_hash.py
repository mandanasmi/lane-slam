import hashlib

def get_md5(contents):
    m = hashlib.md5()
    m.update(contents)
    s = m.hexdigest()
    return s
