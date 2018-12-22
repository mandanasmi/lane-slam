from abc import abstractmethod, ABCMeta
from duckietown_utils.contracts_ import contract

class Person(object):
    __metaclass__ = ABCMeta
        
    @abstractmethod
    def get_tags(self):
        pass
    
    @abstractmethod
    @contract(returns='str')
    def get_name(self):
        pass
    
    @abstractmethod
    @contract(returns='None|str')
    def get_position(self):
        pass 

class PersonV1(Person):
    
    def __init__(self, tags=[], position=None, url=None, 
                 order=None, bio=None, name=None, roster_note=None):
        self.tags = tags
        self.position = position
        self.url = url
        self.order = order
        self.bio = bio
        self.name = name
        self.roster_note = roster_note
        
    def get_tags(self):
        return self.tags
    
    def get_name(self):
        return self.name
    
    def get_position(self):
        return self.position
    

    
