
import os
from fabric.api import *

# TODO: add '/opt/%s-%s' % (env.project, env.environment) to ROS_PATH
# TODO: make sure remote export dir exists before build otherwise make will complain

# ---- FABRIC SETTINGS -----------------------------------------------------------

# Assume user 'root' when executing commands and copying files
env.user = 'robot'
# If some command fails, stop the execution
env.warn_only = 'false'

# ---- PROJECT SETTINGS ---------------------------------------------------------- #

# Project name, will be used as root folder name for deployments
env.project = 'amee'
# Assumes that project structure is using structure <project_root>/<misc> 
# That means we are always running from <project_roo>/deploy
env.root_path = os.path.dirname(__file__).rsplit('/',1)[0] + '/amee'

# # Get a nice timestamp like '20100212-151218' to flag deployments
# env.timestamp = utils.timestamp_mark();

# Host to deploy to
env.hosts = ['robo0']
env.remote_deploy_path = '/home/robot/amee'


# ---- TASK ---------------------------------------------------------------------- #

# USAGE (for full deployment): fab build run_tests deploy activate
# NOTE: During deploy You WANT full build to make sure everything is built using latest sources

@task
def environment():
    print "Will deploy to: %s on hosts: %s" % (env.remote_deploy_path, env.hosts)
    print "Using build from: %s" % (env.root_path)


@task
def start():
    """Tries to start project on target by first killing (possible) running instance and the starting /current"""
    # TODO Kill currently running roscore
    # run("kill -TERM $(ps aux | grep WebServer.pl | grep -v grep | head -1 | awk '{ print $2 }') ; \
    #     export PROBER_CONFIG=%(config_file)s && \
    #      /usr/bin/hypnotoad -c '%(release)s/config/hypnotoad.conf' %(release)s/WebServer.pl" % {
    #     'release' : current_release,
    #     'config_file' : current_release + '/config/p2.conf'
    # })

    # TODO Start some launch script (maybe we want to supply which?)
    # run('%(deploy_dir)s/ProbeServer.pl -r --config=%(config_file)s' % {
    #     'deploy_dir' : current_release,
    #     'config_file' : current_release + "/config/p2.conf",
    # })

@task
def run_tests():
    # TODO
    return

@task
def build():
    """Builds project"""
    require('root_path')
    local('cd %s; rosmake' % (env.root_path))


@task
def deploy():
    """Performs deployment of previously built binaries to target"""
    require('root_path', 'remote_deploy_path')
    
    # Upload
    run('mkdir -p %s' % env.remote_deploy_path)

    # nodes, bin, launch
    rsync_project(
        local_dir='%s/%s' % (env.root_path, 'nodes'),
        remote_dir='%s/%s' % (env.remote_deploy_path, 'nodes'),
        delete='true',
    )
    rsync_project(
        local_dir='%s/%s' % (env.root_path, 'bin'),
        remote_dir='%s/%s' % (env.remote_deploy_path, 'bin'),
        delete='true',
    )
    rsync_project(
        local_dir='%s/%s' % (env.root_path, 'launch'),
        remote_dir='%s/%s' % (env.remote_deploy_path, 'launch'),
        delete='true',
    )






