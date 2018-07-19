#!/usr/bin/env python
import rospy
import logging
from imitation_learning.util import setup_log
from ros_util import ROSLogging
from autorally_expert_mppi import AutoRallyExpertMPPI
from autorally_environment import AutoRally
from imitation_learning.dagger import dagger
from imitation_learning.learners import KerasLearner, TFLearner
from nn import keras_dnn_ws


if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node("imlearn", anonymous=False)

    # Load algorithm parameters
    nb_epoch = rospy.get_param('~nb_epoch')
    pretrain_epoch = rospy.get_param('~pretrain_epoch')
    batch_size = rospy.get_param('~batch_size')
    dim_act = rospy.get_param('~dim_act')
    dim_img = rospy.get_param('~dim_img')
    dim_ws = rospy.get_param('~dim_ws')
    dim_imu = rospy.get_param('~dim_imu')
    pretrain = rospy.get_param('~pretrain')
    rollouts_pretrain = rospy.get_param('~rollouts_pretrain')
    timesteps_pretrain = rospy.get_param('~timesteps_pretrain')
    rollouts_test = rospy.get_param('~rollouts_test')
    timesteps_test = rospy.get_param('~timesteps_test')
    test_expert = rospy.get_param('~test_expert')
    test_initial = rospy.get_param('~test_initial')
    test_policy = rospy.get_param('~test_policy')
    test_final = rospy.get_param('~test_final')
    iterations = rospy.get_param('~iterations')
    rollouts = rospy.get_param('~rollouts')
    timesteps = rospy.get_param('~timesteps')
    shuffle = rospy.get_param('~shuffle')
    model_name = rospy.get_param('~model')
    data = rospy.get_param('~data')
    mixing_rate = rospy.get_param('~mixing_rate')
    use_human = rospy.get_param('~use_human')
    human_delay = rospy.get_param('~human_delay')
    mix_within_rollout = rospy.get_param('~mix_within_rollout')
    clamp_control = rospy.get_param('~clamp_control')
    # ws_smoothing_coef = rospy.get_param('~ws_smoothing_coef')
    # imu_smoothing_coef = rospy.get_param('~imu_smoothing_coef')
    freeze_weights = rospy.get_param('~freeze_weights')
    hz = rospy.get_param('~rate_hz')

    # Expert and environment
    expert = AutoRallyExpertMPPI(control_topic='/mppi_controller/mppi_command',
                                 status_topic='/mppi_controller/mppiStatus',
                                 cost_topic='/mppi_controller/mppiStatus', # There is no cost topic published by MPPI. TODO: Delete this later
                                 autonomous=True)
    env = AutoRally(expert=expert)

    # Load file/folder paths
    log_name = 'imlearn'
    model_dir = rospy.get_param('~model_dir')
    data_dir = rospy.get_param('~data_dir')
    log_dir = rospy.get_param('~log_dir')
    logger = setup_log(log_dir + '/' + log_name)

    # Saved data and model
    if model_name == 'None':
        saved_model = None
    else:
        saved_model = model_dir + '/' + model_name
    if data == 'None':
        saved_data = None
    else:
        saved_data = data_dir + '/' + data

    # Redirect logging to ROS
    logging.getLogger(log_name).addHandler(ROSLogging())
    logging.getLogger(log_name).setLevel(logging.INFO)

    field_names = ['img_left', 'ws']
    field_dims = [dim_img, dim_ws]

    # Algorithm
    model = keras_dnn_ws(dim_img, dim_ws, dim_act)
    learner = KerasLearner(model, field_names, field_dims, log_name=log_name)
    data = dagger(env, expert, learner, timesteps, rollouts, iterations, mixing_rate,
                          loaded_data=saved_data, pretrain=pretrain,
                          test_expert=test_expert, test_initial=test_initial,
                          test_policy=test_policy, test_final=test_final,
                          log_name=log_name, data_save_path=data_dir,
                          model_save_path=model_dir,
                          mix_within_rollout=mix_within_rollout,
                          timesteps_test=timesteps_test,
                          rollouts_test=rollouts_test,
                          timesteps_pretrain=timesteps_pretrain,
                          rollouts_pretrain=rollouts_pretrain,
                          options_pretrain={'batch_size': batch_size,
                                                   'nb_epoch': pretrain_epoch,
                                                   'shuffle': shuffle},
                          options_train={'batch_size': batch_size,
                                          'nb_epoch': nb_epoch,
                                          'shuffle': shuffle})
