#!/usr/bin/env bash
set -e # immediately exit if any command returns non-zero exit code

readonly PROJECT="OpenCatEsp32S3"
readonly SOURCE_DIR="../esp32rs/$PROJECT"

arduino-cli compile --fqbn esp32:esp32:esp32s3 --board-options "CPUFreq=80,DebugLevel=verbose,PartitionScheme=partitions_factory_only" $SOURCE_DIR --output-dir $SOURCE_DIR/build

# --build-property "build.extra_flags=-DCORE_DEBUG_LEVEL=5"
#  --build-property "build.partitions=huge_app"

# Added the partitions_factory_only.csv to /Users/jay/Library/Arduino15/packages/esp32/hardware/esp32/2.0.12/tools/partitions/

# Added the following lines in /Users/jay/Library/Arduino15/packages/esp32/hardware/esp32/2.0.12/boards.txt:
# esp32s3.menu.PartitionScheme.partitions_factory_only=partitions_factory_only
# esp32s3.menu.PartitionScheme.partitions_factory_only.build.partitions=partitions_factory_only
### Maybe set the maximum size as well?
# esp32s3.menu.PartitionScheme.partitions_factory_only.upload.maximum_size=3145728

xtensa-esp32-elf-nm -n $SOURCE_DIR/build/OpenCatEsp32S3.ino.elf > $SOURCE_DIR/build/symbols.txt


# Initial build files
readonly OUT_DIR="www/static/$PROJECT/build/"
mkdir -p $OUT_DIR
cp $SOURCE_DIR/build/${PROJECT}.ino.bootloader.bin $OUT_DIR
cp $SOURCE_DIR/build/${PROJECT}.ino.partitions.bin $OUT_DIR
cp $SOURCE_DIR/build/${PROJECT}.ino.bin $OUT_DIR
cp $SOURCE_DIR/build/symbols.txt $OUT_DIR
cp $SOURCE_DIR/bootloader_symbols.txt $OUT_DIR
cp -r ../esp32rs/rom www/static/

# Display files
cp $SOURCE_DIR/${PROJECT}.ino www/src/assets/$PROJECT/
cp -r $SOURCE_DIR/src www/src/assets/$PROJECT/

# default compiled binary and symbol for when user hits reset
cp $SOURCE_DIR/build/${PROJECT}.ino.bin www/src/assets/
cp $SOURCE_DIR/build/symbols.txt www/src/assets/
