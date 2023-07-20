import os 
import platform

if platform.system() == "Windows":
    os.environ["path"] += ";C:\\msys64\\mingw64\\bin;C:\\msys64\\ucrt64\\bin;C:\\msys64\\usr\\bin"


