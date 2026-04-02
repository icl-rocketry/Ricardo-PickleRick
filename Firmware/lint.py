Import("env")
env.AddPreAction("buildprog", "clang-tidy -p .pio/build/v3 src/**/*.cpp")