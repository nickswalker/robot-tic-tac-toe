#include <tic_tac_toe/sound.h>
#include <dirent.h>
#include <vector>
#include <ros/package.h>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <sys/stat.h>

using namespace std;
const string FILES_PATH = ros::package::getPath("tic_tac_toe") + "/sounds";
const string CMD = "mplayer";

// Scan wav files
int scan(const string &dir, vector<string> &files) {
    DIR* dr = opendir(dir.c_str());
    struct dirent *drp;

    while ((drp = readdir(dr)) != NULL) {
        struct stat s;
        stat((dir + "/" + string(drp->d_name)).c_str(), &s);

        if (s.st_mode & S_IFREG) {
            files.push_back(string(drp->d_name));
        }
    }

    closedir(dr);

    return 0;
}

//Open vlc
int play_file(const string &file_name) {
    string cmd = CMD;
    vector<string> files, vfiles;

    scan(FILES_PATH, files);

    for (unsigned int i = 0; i < files.size(); i++) {
        // if (files[i].substr(files[i].find(".")) == ".m4a")
        if (files[i] == file_name) {
            vfiles.push_back(string(files[i]));
        }
    }

    cmd += " " + FILES_PATH + "/" + vfiles[0] + " > /dev/null";


    printf("%s\n", cmd.c_str());
    system(cmd.c_str());

    return 0;
}

void play_file_non_blocking(const string &file_name) {
    // play sound concurrently
    pid_t pid = fork();
    if (pid == 0) {
        play_file(file_name);
        exit(0);
    }

}


