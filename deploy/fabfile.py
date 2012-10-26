
import os
from fabric.api import *
from fabric.contrib.project import rsync_project

# TODO: add '/opt/%s-%s' % (env.project, env.environment) to ROS_PATH
# TODO: make sure remote export dir exists before build otherwise make will complain

# ---- FABRIC SETTINGS -----------------------------------------------------------

# Assume user 'root' when executing commands and copying files
env.user = 'robot'
env.password = 'p1ckles'
#env.key_filename = ["/home/robot/.ssh/id_rsa.robo0"]

# If some command fails, stop the execution
env.warn_only = 'false'

# ---- PROJECT SETTINGS ---------------------------------------------------------- #

# Project name, will be used as root folder name for deployments
env.project = 'amee'
# Assumes that project structure is using structure <project_root>/<misc> 
# That means we are always running from <project_roo>/deploy
env.root_path = os.path.dirname(__file__).rsplit('/',1)[0]

env.remote_deploy_path = '/home/robot/ros_workspace/robot12'

# Host to deploy to
env.hosts = ['robo0']


# ---- TASK ---------------------------------------------------------------------- #

# USAGE (for full deployment): fab build run_tests deploy activate
# NOTE: During deploy You WANT full build to make sure everything is built using latest sources


@task
def run_tests():
    print '------------ RUNNING TEST SUITE ------------'
    print "All tests OK"

@task
def build():
    """Builds project"""
    print '------------ BUILDING PROJECT ------------'
    require('root_path')
    local('cd %s/amee; rosmake' % (env.root_path))


@task
def deploy():
    """Performs deployment of previously built binaries to target"""
    
    #build()
    #run_tests()

    print '------------ DEPLOYING PROJECT ------------'
    require('root_path', 'remote_deploy_path')

    # Upload
    #run('mkdir -p %s/amee' % env.remote_deploy_path)

    # Sync custom bin's
    rsync_project(
        local_dir='%s/bin' % (env.root_path),
        remote_dir='%s/' % (env.remote_deploy_path),
    )

    rsync_project(
        local_dir='%s/amee' % (env.root_path),
        remote_dir='%s' % (env.remote_deploy_path),
        exclude = 'build',
    )

    # # Sync ROS project files
    # ros_files = ['manifest.xml', 'mainpage.dox']
    # for f in ros_files:
    #     rsync_project(
    #         local_dir='%s/amee/%s' % (env.root_path, f),
    #         remote_dir='%s/amee/' % (env.remote_deploy_path),
    #     )

    # # Sync ROS project dirs
    # ros_dirs = ['amee/bin', 'amee/launch', 'amee/nodes', 'amee/msg_gen','amee/msg', 'amee/src']
    # for cur_dir in ros_dirs:
    #     rsync_project(
    #         local_dir='%s/%s' % (env.root_path, cur_dir),
    #         remote_dir='%s/amee' % (env.remote_deploy_path),
    #     )


@task
def list_launch():
    """List launch scripts available on robot"""
    run('ls %(launch_dir)s/*.launch' % {
            'launch_dir' : env.remote_deploy_path + '/amee/launch',
        })

@task
def launch(target):
    """Tries to start project on target by first killing (possible) running instance and the starting /current"""
    
    print '------------ LAUNCHING ------------'

    # TODO Kill (possible) currently running roscore first

    # run("kill -TERM $(ps aux | grep WebServer.pl | grep -v grep | head -1 | awk '{ print $2 }') ; \
    #     export PROBER_CONFIG=%(config_file)s && \
    #      /usr/bin/hypnotoad -c '%(release)s/config/hypnotoad.conf' %(release)s/WebServer.pl" % {
    #     'release' : current_release,
    #     'config_file' : current_release + '/config/p2.conf'
    # })

    # TODO Start some launch script (maybe we want to supply which?)
    run('roslaunch %(deploy_dir)s/amee/launch/%(launch_file)s' % {
        'deploy_dir' : env.remote_deploy_path,
        'launch_file' : target,
    })



