import { initFiles, FileEntry } from "chimpanzee-ui";

// Load the OpenCatEsp32S3 sources into the shared file store
export function initProjectFiles() {
  // Arguments: (Directory, Search Subdirectories?, Regex to match files)
  const filesContext = require.context(
    "./assets/OpenCatEsp32S3",
    true,
    /\.(cpp|c|h|hpp|ino)$/i,
  );

  const entries: Record<string, FileEntry> = {};
  filesContext
    .keys()
    // Arduino only builds .ino files at the sketch root; skip nested ones (e.g. src/display_test.ino)
    .filter((key) => !/^\.\/.+\/[^/]+\.ino$/i.test(key))
    .sort((a, b) => {
      return a.localeCompare(b);
    })
    .forEach((key) => {
      // 'key' looks like: "./main.cpp" or "./utils/config.h"
      // filesContext(key) returns the raw string content (via the asset/source rule)
      entries[key.replace(/^\.\//, "")] = {
        content: filesContext(key),
        language: "cpp",
      };
    });

  initFiles(entries);
}
