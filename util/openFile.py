# Created by trevorlong on 2/14/25
# license
# Copyright 2025 trevorlong

# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
# documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
# and to permit persons to whom the Software is furnished to do so.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
# THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import pathlib as plb


def readFile(filepath:plb.Path)->[str,list]:
    if not (filepath.exists()):
        raise FileNotFoundError(f"File {str(filepath)} does not exist")

    with open(str(filepath), "r") as f:
        lines = f.readlines()
        fulltxt = f.read()

    return fulltxt,lines


if __name__=="__main__":
    example_file_path = plb.Path("../fitting/example-files/b737.run")
    a,b = readFile(example_file_path)
    print(type(a))
    print(type(b))
