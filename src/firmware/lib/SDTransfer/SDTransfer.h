/*
 * SDTransfer.h — SD 카드 파일 전송 라이브러리 (USB Serial 경유)
 *
 * 프로토콜:
 *   ls             → __FILE_LIST_START__ / name,size / __FILE_LIST_END__
 *   get <filename> → __FILE_START__,name,size / binary / __FILE_END__
 *   del <filename> → __DELETED__: name  or  __ERROR__: msg
 *
 * 사용법:
 *   #include <SDTransfer.h>
 *
 *   void handleCommand(String cmd) {
 *     if (SDTransfer::handleCommand(cmd, isLogging)) return;
 *     // ... 나머지 명령어 ...
 *   }
 */

#ifndef SD_TRANSFER_H
#define SD_TRANSFER_H

#include <Arduino.h>
#include <SD.h>

class SDTransfer {
public:
    /// 명령어 처리. isLogging=true이면 get/del을 거부.
    /// @return true = SD 명령을 처리함, false = SD 명령이 아님
    static bool handleCommand(const String& cmd, bool isLogging = false);

private:
    static void listFiles();
    static void getFile(const String& filename);
    static void deleteFile(const String& filename);
};

#endif // SD_TRANSFER_H
