#!/usr/bin/python

import numpy as np
from scipy import signal

class RandomMapGenerator():

    def __init__(self,size=(5,6)):
        self.initialize_thresh = 0.5
        self.size = size


    def initialize(self):
        self.map = np.random.random(self.size)
        super_thresh_indices = self.map >= self.initialize_thresh
        sub_thresh_indices = self.map < self.initialize_thresh
        self.map[super_thresh_indices] = 1
        self.map[sub_thresh_indices] = -1

    def apply_rules(self):
        # pad the map with -1's around the outside (i.e. a row of non-road around the outside)
        map_padded = np.lib.pad(self.map,((1,1),(1,1)),'constant',constant_values=((-1,-1),(-1,-1)))
        # TODO read from yaml and put the masks into a loop
        mask = np.array([[1,1],[1,1]])
        if not self.check_mask(mask,map_padded):
            return False
        mask = np.array([[0,-1,0],[-1,1,-1]])
        if not self.check_mask(mask,map_padded):
            return False
        mask = np.array([[-1,0],[1,-1],[-1,0]])
        if not self.check_mask(mask,map_padded):
            return False 
        mask = np.array([[-1,1,-1],[0,-1,0]])
        if not self.check_mask(mask,map_padded):
            return False
        mask = np.array([[0,-1],[-1,1],[0,-1]])
        if not self.check_mask(mask,map_padded):
            return False
        return True
        
    def check_mask(self,mask,map_padded):
        # the maximum correlation value will equal the number of non-zero vals
        check_val = np.count_nonzero(mask)
        masked = signal.correlate(map_padded,mask)
        if (np.amax(masked) == check_val):
            return False
        return True
    

    def generate(self):
        while True:
            self.initialize()
            if self.apply_rules():
                break
            
        return self.map;

def main():
    size=(4,4)
    rmg = RandomMapGenerator(size)
    map = rmg.generate()
    print map

    
if __name__ == "__main__":
    main()
