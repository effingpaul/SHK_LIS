#include <iostream>
#include <OptiTrack/optitrack.h>
#include <Core/graph.h>
#include <KOMO/komo.h>
#include <BotOp/bot.h>
#include <Optim/NLP_Solver.h>

int main(int argc,char **argv) {
    rai::initCmdLine(argc, argv);

    rai::Configuration C;
    C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
    C.view(false);

    BotOp bot(C, true);
    bot.home(C);

    // await key input 'k' to proceed
    cout << "Press enter to start teleoperation" << endl;
    std::cin.get();

    bot.gripperClose(rai::_left);
    // END OWN CODE
}