

class RosLogs(object):
    """

    Class to save the logs in ROS to several files

    """

    def __init__(self, path, prefix, frequency, do_log=True):
        """
        Constructor

          path - Path to where the logs will be saved
          prefix - Prefix for the logs
          frequency - value used for rospy.spin
          do_log - log the data to the respective files (default True)

        """
        self.path = path+prefix
        self.dt = 1.0/frequency
        self.do_log = do_log
        # Initializations
        self.time = 0
        self.task = []
        self.taskdim = []
        self.ftask = []
        self.ftaskdes = []
        # Log poses
        self.flinkpose = []
        self.linkname = []
        if (do_log):
            # Joint configurations are always logged
            self.fjoints = open(self.path+"q.txt", "w")
            # Log for computation time
            self.ftime = open(self.path+"time.txt", "w")
            self.save = self._save
            self.savetime = self._savetime
        else:
            # Do nothing if no log is needed
            self.save = self._save_dummy
            self.savetime = self._savetime_dummy


    def addTask(self, task, name=None):
        """
        Add a task to log the sensed and desired value
          task - task object
          name - file name (if not set, the task name is used)

        """
        if (name==None):
            tname = task.ctask.getName()
        else:
            tname = name
        if (self.do_log):
            # Only open the files if log is needed
            fl = open(self.path + tname + ".txt", "w")
            fldes = open(self.path + tname + "_des.txt", "w")
            self.ftask.append(fl)
            self.ftaskdes.append(fldes)
        tdim  = task.getTaskDim()
        self.task.append(task)
        self.taskdim.append(tdim)


    def addPoses(self, linknames, simrobot, lognames=[]):
        """
        Add poses to log their current value
          linkname - list of link names ['l1', 'l2'], or a single string
          simrobot - sim.robot object
          lognames - list of file names (if not set, the link names are used)

        """
        self.robot = simrobot
        if (lognames==[]):
            pnames = linknames
        else:
            pnames = lognames
        if (self.do_log):
            # Only open the files if log is needed
            if linknames.__class__ == str:
                fpose = open(self.path + pnames + ".txt", "w")
                self.flinkpose.append(fpose)
                self.linkname.append(linknames)
            else:
                for i in range(len(linknames)):
                    fpose = open(self.path + pnames[i] + ".txt", "w")
                    self.flinkpose.append(fpose)
                    self.linkname.append(linknames[i])
        

    def _save(self, q):
        """
        Save the logs for the joints and the tasks. Do not use this method
        directly, use 'self.save' instead.

          q - joint configuration

        """
        # Save the joint configuration
        self.fjoints.write(str(self.time) + ' ')
        if (len(q.shape)==1):
            for i in xrange(q.shape[0]):
                self.fjoints.write(str(q[i])+' ')
        else:
            for i in xrange(q.shape[0]):
                self.fjoints.write(str(q[i,0])+' ')
        self.fjoints.write('\n')
        # Save sensed task values
        for k in xrange(len(self.task)):
            x    = self.task[k].getSensedValue()
            self.ftask[k].write(str(self.time) + ' ')
            for i in xrange(len(x)):
                self.ftask[k].write(str(x[i,0])+' ')
            self.ftask[k].write('\n')
        # Save desired task values
        for k in xrange(len(self.task)):
            xdes = self.task[k].getDesiredValue()
            if (len(xdes)==0):
                pass
            else:
                self.ftaskdes[k].write(str(self.time) + ' ')
                for i in xrange(len(xdes)):
                    self.ftaskdes[k].write(str(xdes[i,0])+' ')
                self.ftaskdes[k].write('\n')
        # Save the poses, if needed
        for k in xrange(len(self.linkname)):
            x = self.robot.linkPose(self.linkname[k])
            self.flinkpose[k].write(str(self.time) + ' ')
            for i in xrange(7):
                self.flinkpose[k].write(str(x[i,0])+' ')
            self.flinkpose[k].write('\n')
        # Update the time
        self.time += self.dt


    def _savetime(self, tic, toc, show=False):
        """
        Save the difference in time between tic and toc in ms. Do not use this
        method directly, use 'self.savetime' instead.

          tic - initial time (us), manually obtained with time.time()
          toc - final time (us), manually obtained with time.time()
          show - show in the screen the elapsed time in ms

        """
        time = 1000*(toc-tic) # in ms
        self.ftime.write(str(time) + '\n')
        if (show):
            print 'Elapsed time:', time, 'ms'


    def _save_dummy(self, q):
        """
        Do nothing (only update time). Used internally when do_log is set
        to False

        """
        # Update the time
        self.time += self.dt


    def _savetime_dummy(self, tic, toc, show=False):
        """
        Do nothing. Used internally when do_log is set to False

        """
        pass


    def close(self):
        """
        Close all the log files

        """
        if (self.do_log):
            print "... closing the log files"
            self.fjoints.close()
            self.ftime.close()
            for k in xrange(len(self.task)):
                self.ftask[k].close()
                self.ftaskdes[k].close()
        else:
            pass


    def printTime(self):
        """
        Print the internal time on the screen

        """
        print ' t:', self.time
        

