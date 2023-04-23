from sdf_generator import *
import numpy as np
import warnings
import argparse


class CHIATModelCreator(object):
    def __init__(self, **kwargs):
        self.name = "KITE_TMP_MDL"
        self.folder = "../models/"


        if "name" in kwargs:
            self.name = kwargs["name"]

        self.sdfgen = SDFGenerator(name=self.name, folder=self.folder)
        self.sdfgen.generateConfigFile()
        self.sdfgen.open()
        
        pass

    def __del__(self):
        self.sdfgen.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='KITE Model Creator')
    parser.add_argument('--name', type=str, default="KITE_TMP_MDL", help='Name of the model')
    args = parser.parse_args()

    kite = CHIATModelCreator(name=args.name)
    pass
