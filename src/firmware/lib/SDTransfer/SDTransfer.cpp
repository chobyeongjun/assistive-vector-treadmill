/*
 * SDTransfer.cpp — SD 카드 파일 전송 라이브러리 구현
 */

#include "SDTransfer.h"

bool SDTransfer::handleCommand(const String& cmd, bool isLogging) {
    if (cmd == "ls") {
        listFiles();
        return true;
    }
    if (cmd.startsWith("get ")) {
        if (isLogging) {
            Serial.println("__ERROR__: Stop logging first");
            return true;
        }
        String fname = cmd.substring(4);
        fname.trim();
        getFile(fname);
        return true;
    }
    if (cmd.startsWith("del ")) {
        if (isLogging) {
            Serial.println("__ERROR__: Stop logging first");
            return true;
        }
        String fname = cmd.substring(4);
        fname.trim();
        deleteFile(fname);
        return true;
    }
    return false;
}

void SDTransfer::listFiles() {
    File root = SD.open("/");
    if (!root) {
        Serial.println("__SD_ERROR__");
        return;
    }
    Serial.println("__FILE_LIST_START__");
    while (File entry = root.openNextFile()) {
        if (!entry.isDirectory()) {
            Serial.print(entry.name());
            Serial.print(",");
            Serial.println(entry.size());
        }
        entry.close();
    }
    root.close();
    Serial.println("__FILE_LIST_END__");
}

void SDTransfer::getFile(const String& filename) {
    File f = SD.open(filename.c_str(), FILE_READ);
    if (!f) {
        Serial.println("__ERROR__: File not found");
        return;
    }
    Serial.print("__FILE_START__,");
    Serial.print(filename);
    Serial.print(",");
    Serial.println(f.size());

    uint8_t buf[4096];
    while (f.available()) {
        size_t n = f.read(buf, sizeof(buf));
        Serial.write(buf, n);
    }
    f.close();
    Serial.println("\n__FILE_END__");
}

void SDTransfer::deleteFile(const String& filename) {
    if (SD.remove(filename.c_str())) {
        Serial.print("__DELETED__: ");
        Serial.println(filename);
    } else {
        Serial.println("__ERROR__: Delete failed");
    }
}
