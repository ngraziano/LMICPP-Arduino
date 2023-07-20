
# ugly but easy to make the test pass : copy the dlls to the folder of the exe
# copy libgcc_s_seh-1 libstdc++-6 libwinpthread-1 to the folder of the exe

import platform
import shutil

Import("env")


msys_dlls = ["libgcc_s_seh-1.dll", "libstdc++-6.dll", "libwinpthread-1.dll"]

if platform.system() == "Windows":
    for dll in msys_dlls:
        source = "C:\\msys64\\ucrt64\\bin\\" + dll
        dest = env.subst('$BUILD_DIR\\')  + dll
        shutil.copyfile(source, dest)


