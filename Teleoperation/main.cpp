#include <iostream>
#include <OptiTrack/optitrack.h>
#include <Core/graph.h>
#include <KOMO/komo.h>
#include <BotOp/bot.h>
#include <Optim/NLP_Solver.h>
#define BOTH_ARMS 0

arr quaternionRotation(arr q1, arr q2) {
    arr q = {0., 0., 0., 0.};
    q.elem(0) = q1.elem(0) * q2.elem(0) - q1.elem(1) * q2.elem(1) - q1.elem(2) * q2.elem(2) - q1.elem(3) * q2.elem(3);
    q.elem(1) = q1.elem(0) * q2.elem(1) + q1.elem(1) * q2.elem(0) + q1.elem(2) * q2.elem(3) - q1.elem(3) * q2.elem(2);
    q.elem(2) = q1.elem(0) * q2.elem(2) - q1.elem(1) * q2.elem(3) + q1.elem(2) * q2.elem(0) + q1.elem(3) * q2.elem(1);
    q.elem(3) = q1.elem(0) * q2.elem(3) + q1.elem(1) * q2.elem(2) - q1.elem(2) * q2.elem(1) + q1.elem(3) * q2.elem(0);
    return q;
    //       a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,  // 1
    //    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,  // i
    //    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,  // j
    //   a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w   // k
}


void reload_target(rai::Configuration* C, arr target_origin, arr controller_origin, arr rotation_offset, const char* controller, const char* gripper_target) {
    arr controller_quat = C->getFrame(controller)->getQuaternion();

    auto quat_tmp = controller_quat.elem(1);
    controller_quat.elem(1) = -controller_quat.elem(3);
    controller_quat.elem(3) = quat_tmp;

    quat_tmp = controller_quat.elem(2);
    controller_quat.elem(2) = controller_quat.elem(3);
    controller_quat.elem(3) = -quat_tmp;

    float tmpW = controller_quat.elem(0);
    float tmpX = controller_quat.elem(1);
    float tmpY = controller_quat.elem(2);
    float tmpZ = controller_quat.elem(3);

    // rotate the quaternion so that the coordinate system is rotated around the x axis by 90 degrees
    // this is necessary because the controller is rotated by 90 degrees around the x axis
    // this is done by multiplying the quaternion with a quaternion that represents a 90 degree rotation around the x axis
    arr rotationAroundX = {sqrt(2)/2, 0,  sqrt(2)/2, 0};
    controller_quat = quaternionRotation(rotationAroundX, controller_quat);

    

    C->getFrame("controller indicator")->setQuaternion(controller_quat);
    C->getFrame(gripper_target)->setQuaternion(controller_quat);
    arr controller_pos = C->getFrame(controller)->getPosition() - controller_origin;
    auto tmp_pos = controller_pos.elem(0);
    controller_pos.elem(0) = -controller_pos.elem(1); // QUESTION: why this reassignemnt?
    controller_pos.elem(1) = tmp_pos;
    // OWN CODE
    float SCALING_FACTOR = 2;
    // END OWN CODE
    arr target_pos = target_origin + controller_pos*SCALING_FACTOR; // This is the factor to scale the movements
    C->getFrame(gripper_target)->setPosition(target_pos);
}

bool GRIPPER_CONTROL = false; // OWN CODE
auto VELOCITY = 1.; //OWN CODE

int main(int argc,char **argv) {
    rai::initCmdLine(argc, argv);

    rai::Configuration C;
    C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
    C.view(false);

    BotOp bot(C, true);
    bot.home(C);
    //bot.gripperMove(rai::_left, .079);
    #if BOTH_ARMS
    bot.gripperMove(rai::_right, .079);
    #endif

    // OWN CODE
    // await key input 'k' to proceed
    cout << "Press enter to start teleoperation" << endl;
    std::cin.get();

    bot.gripperClose(rai::_left);
    // END OWN CODE

    const char* to_follow = "l_hand";
    // OWN CODE
    to_follow = "stick_controller";
    // END OWN CODE

    rai::OptiTrack OT;
    OT.pull(C);

    bool activated = false;
    bool l_gripper_closed = false;
    // bool r_gripper_closed = false;

    #if BOTH_ARMS
    arr r_controller_origin = arr{0., 0., 0.};
    arr r_target_origin = arr{0., 0., 0.};
    arr r_rotation_offset = arr{0., 0., 0., 1.};
    #endif

    arr l_controller_origin = arr{0., 0., 0.};
    arr l_target_origin = arr{0., 0., 0.};
    arr l_rotation_offset = arr{0., 0., 0., 1.};

    while(1) {
        OT.pull(C);

        if(C.view(false)=='q') break;
        if (C.view(false)=='k' && !activated) {
            activated = true;

            #if BOTH_ARMS
            r_controller_origin = C.getFrame("r_controller")->getPosition();
            r_target_origin = C.getFrame("r_gripper")->getPosition();
            r_rotation_offset = C.getFrame("r_controller")->getQuaternion();
            C.addFrame("r_gripper_target")->setPosition(r_target_origin).setShape(rai::ST_marker, {.2});
            #endif

            l_controller_origin = C.getFrame(to_follow)->getPosition();
            arr l_controller_quat = C.getFrame(to_follow)->getQuaternion(); // OWN CODE FOR VISUALIZATION
            l_target_origin = C.getFrame("l_gripper")->getPosition();
            arr l_target_quat = C.getFrame("l_gripper")->getQuaternion(); // OWN CODE FOR VISUALIZATION
            l_rotation_offset = C.getFrame(to_follow)->getQuaternion();
            C.addFrame("l_gripper_target")->setPosition(l_target_origin).setShape(rai::ST_marker, {.01});
            C.addFrame("controller indicator")->setQuaternion(l_controller_quat).setPosition(l_controller_origin).setShape(rai::ST_marker, {.8}); // OWN CODE FOR VISUALIZATION
            C.addFrame("gripper_marker")->setQuaternion(l_target_quat).setPosition(l_target_origin).setShape(rai::ST_marker, {.2}); // OWN CODE FOR VISUALIZATION
        }

        // Define KOMO problem towards the target waypoint
        if (activated) {

            #if BOTH_ARMS
            reload_target(&C, r_target_origin, r_controller_origin, r_rotation_offset, "r_controller", "r_gripper_target");
            #endif
            reload_target(&C, l_target_origin, l_controller_origin, l_rotation_offset, to_follow, "l_gripper_target");

            KOMO komo(C, 1., 1, 2, true);
            komo.addControlObjective({}, 0, 1e1);
            komo.addControlObjective({}, 1, .1);
            komo.addControlObjective({}, 2, .1);
            komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e10});
            komo.addObjective({}, FS_jointLimits, {}, OT_ineq, {1e0});

            bool l_should_translate = -C.eval(FS_negDistance, {"l_gripper", "l_gripper_target"}).elem(0) > .01;
            if (l_should_translate) {
                komo.addObjective({1.}, FS_positionDiff, {"l_gripper", "l_gripper_target"}, OT_eq, {1e1});
            } else {
                komo.addObjective({}, FS_position, {"l_gripper"}, OT_eq, {1e1}, C.getFrame("l_gripper")->getPosition());
            }

            #if BOTH_ARMS
            bool r_should_translate = -C.eval(FS_negDistance, {"r_gripper", "r_gripper_target"}).elem(0) > .01;
            if (r_should_translate) {
                komo.addObjective({1.}, FS_positionDiff, {"r_gripper", "r_gripper_target"}, OT_eq, {1e1});
            } else {
                komo.addObjective({}, FS_position, {"r_gripper"}, OT_eq, {1e1}, C.getFrame("r_gripper")->getPosition());
            }
            #else
            bool r_should_translate = false;
            #endif

            #if BOTH_ARMS
            komo.addObjective({1.}, FS_quaternionDiff, {"r_gripper", "r_gripper_target"}, OT_eq, {1e1});
            #endif
            komo.addObjective({1.}, FS_quaternionDiff, {"l_gripper", "l_gripper_target"}, OT_eq, {1e1});

            if (r_should_translate || l_should_translate) {
                auto ret = NLP_Solver()
                    .setProblem(komo.nlp())
                    .solve();
                cout << *ret <<endl;
                arr q = komo.getPath_qOrg();
                bot.moveTo(q[0], {VELOCITY}, true); // velo is second param
                bot.sync(C, 0);
            }

            #if GRIPPER_CONTROL

                float l_gripper_pos = .079 - (C.eval(FS_negDistance, {"l_controller", "l_thumb"}).elem(0)+.014) / -.07 * .079; // QUESTION: Where do these numbers come from?
                if (l_gripper_pos > .079) l_gripper_pos = .079;
                if (l_gripper_pos < .0) l_gripper_pos = .0;
                // std::cout << "Left gripper pos: " << l_gripper_pos << std::endl;

                if (l_gripper_closed && l_gripper_pos >= .079*.66) {
                    bot.gripperMove(rai::_left, .079);
                    l_gripper_closed = false;
                }
                else if (!l_gripper_closed && l_gripper_pos <= .079*.33) {
                    bot.gripperClose(rai::_left);
                    l_gripper_closed = true;
                }

                #if BOTH_ARMS
                float r_gripper_pos = .079 - (C.eval(FS_negDistance, {"r_controller", "r_thumb"}).elem(0)+.014) / -.07 * .079;
                if (r_gripper_pos > .079) r_gripper_pos = .079;
                if (r_gripper_pos < .0) r_gripper_pos = .0;
                std::cout << "Right gripper pos: " << r_gripper_pos << std::endl;

                if (r_gripper_closed && r_gripper_pos >= .079*.66) {
                    bot.gripperMove(rai::_right, .079);
                    l_gripper_closed = false;
                }
                else if (!r_gripper_closed && r_gripper_pos <= .079*.33) {
                    bot.gripperClose(rai::_right);
                    r_gripper_closed = true;
                }
                #endif
            #endif
        }
    }

    return 0;
}
