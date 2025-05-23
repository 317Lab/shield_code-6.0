Import("env")
import os

if env.IsIntegrationDump():
   # stop the current script execution
   Return()

# code below runs for the "build" and other targets
env.Append(CPPDEFINES=["MACRO_NAME"])


libs = env["LIBS"]
#print("Current libs: " + str(libs))
# remove the original "sam_sam3x8e_gcc_rel" lib
libs = [l for l in libs if l != "sam_sam3x8e_gcc_rel"]
env.Replace(LIBS = libs)
# inject our modified one which has system_sam3xa.o removed
env.Append(
    LIBS=[
        File(os.path.join(env.subst("$PROJECT_DIR"), "modded_libsam_sam3x8e_gcc_rel.a"))
    ]
)
#print("Replaced libs: " + str(env["LIBS"]))
