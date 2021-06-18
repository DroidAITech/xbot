#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "nlp/nlp_feedback.h"

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cout << "please enter the your question\n";
    return -1;
  }
  TuLingRobot tuling;
  std::string tuling_key = "b4aac2a556b042269e418c78aa88f4cc";
  tuling.setAskJson(tuling_key, argv[1]);
  tuling.callTulingApi();
  tuling.textFromJson();
  return 0;
}
