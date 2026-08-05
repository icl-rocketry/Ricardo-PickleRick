Import("env")

from datetime import datetime
from pathlib import Path


env.Append(CXXFLAGS=["-frtti"])

# Generate the log-directory build identifier on every PlatformIO invocation.
# Only system.cpp includes this header, so an incremental build recompiles that
# translation unit without requiring a full clean build.
generated_include_dir = Path(env.subst("$BUILD_DIR")) / "generated"
generated_include_dir.mkdir(parents=True, exist_ok=True)

build_id = datetime.now().astimezone().strftime("%Y-%m-%d_%H-%M-%S")
build_id_header = generated_include_dir / "firmware_build_id.h"
header_contents = (
    "#pragma once\n\n"
    f'#define FIRMWARE_BUILD_ID "{build_id}"\n'
)

if not build_id_header.exists() or build_id_header.read_text() != header_contents:
    build_id_header.write_text(header_contents)

env.Append(CPPPATH=[str(generated_include_dir)])
