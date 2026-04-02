Import("env")
env.AddPreAction("buildprog", "clang-tidy src/**/*.cpp -- -Iinclude -Isrc")