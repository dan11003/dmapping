

int main(int argc, char** argv)
{
    cout << "start calib" << endl;
    ros::init(argc, argv, "calib");
    ImageProjection IP;
    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
    cout << "End calib" << endl;

    return 0;
}
