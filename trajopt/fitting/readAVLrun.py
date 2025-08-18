import pathlib
import re
import sys

import numpy as np
import pathlib as plb
import json


def readAVLrun(file:str or pathlib.Path,
               outputfilename: str or pathlib.Path = "run_output",
               write_permission: str = "newfile",
               ) -> dict:
    """
    Reads AVL runfile and returns a multi-level dict giving the operating point information
    :param file:
    :return:
    """
    # overhead
    file = pathlib.Path(file)

    # ======================================================================
    # check if the file exists and read
    if not(file.exists()):
        raise FileNotFoundError(f"File {file} does not exist")

    with open(file,"r") as f:
        lines = f.readlines()
        fulltxt = f.read()

    # =====================
    # parsing
    conditions = dict()
    for idx, line in enumerate(lines[15:-1]):
        if len(line) < 10:
            continue
        line = re.sub("=","",line)
        linevals = re.split("\\s+",line)
        linevals = list(filter(lambda x: x!='', linevals))
        try:
            conditions[linevals[0]] = float(linevals[1])
        except:
            conditions[linevals[0]] = linevals[1]
        try:
            conditions[f"{linevals[0]}_unit"] = linevals[2]
        except IndexError:
            conditions[f"{linevals[0]}_unit"] = '-'

    # set up outputfile
    if write_permission == "newfile":
        idx = 1
        outputfilename_temp = str(outputfilename)
        while plb.Path(outputfilename_temp + ".json").exists():
            outputfilename_temp = str(outputfilename) + "_" + str(idx)
            idx += 1
            if idx > 1000:
                break
        outputfilename = outputfilename_temp + ".json"
    else:
        outputfilename = str(outputfilename) + ".json"

    # save file
    with open(outputfilename, "w") as f:
        json_text = json.dumps(conditions)
        f.write(json_text)

    return conditions

if __name__ == "__main__":
    readAVLrun("example-files/b737.run")