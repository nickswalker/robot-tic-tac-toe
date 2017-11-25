#ifndef TIC_TAC_TOE_SOUND_H
#define TIC_TAC_TOE_SOUND_H

#include <string>

int scan();
int play_file(const std::string &file_name);
void play_file_non_blocking(const std::string &file_name);

#endif //TIC_TAC_TOE_SOUND_H
