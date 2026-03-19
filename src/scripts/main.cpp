#include "stt-for-diff-drive/turtlebot_node.hpp"

int main(int argc, char **argv)
{
    init(argc, argv);
    spin(make_shared<TurtleBotController>());
    shutdown();
    return 0;
}
