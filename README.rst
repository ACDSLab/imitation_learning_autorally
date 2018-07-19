============================
AutoRally Imitation Learning
============================

**imlearn_autorally** is a ROS package that uses the `imlearn imitation learning library <https://github.gatech.edu/ksaigol3/imitation_learning.git>`_ for end-to-end and imitation learning on the `AutoRally <http://autorally.github.io>`_ platform.

ACDS Development
================

**imlearn_autorally** uses `Git submodules <http://www.vogella.com/tutorials/GitSubmodules/article.html>`_ to pull in the `imlearn <https://github.gatech.edu/ksaigol3/imitation_learning.git>`_ repository.  It tracks the **devel** branch by default.

Using submodules
----------------

The use of submodules means that the following commands should be used instead of/in addition to the usual ``git`` commands.

Clone:
    ``git clone --recursive https://github.gatech.edu/ksaigol3/imlearn_autorally.git``

Update submodule (from the parent folder):
    ``git submodule update --remote``

* Update and merge remote changes on top of local changes:
    ``git submodule update --remote --merge``
* Rewind changes, apply remote changes, then replay local changes:
    ``git submodule update --remote --rebase``

Push the parent repository **without** pushing changes in the imlearn submodule (from the parent folder):
    ``git push``

Prevent pushing changes to the parent repository if changes to the submodules have not been pushed:
    ``git push --recurse-submodules=check``

Attempt to push submodules (if needed) before pushing parent:
    ``git push --recurse-submodules=on-demand``

Change the submodule branch being tracked:
    ``cd src/imitation_learning; git checkout <branch-name>``

Adding executables (ROS nodes, tests, executable scripts)
---------------------------------------------------------

Nodes, tests, and other Python scripts that are executable should be:

* put into the correct directory (usually ``tests/`` or ``scripts/``)

* marked as executable (``chmod +x <filename>``)

* listed in the ``scripts`` list in ``setup.py``

**DO NOT** run ``setup.py`` manually.  It will be called by ``catkin`` whenever you run ``catkin_make``.  Remember to ``source devel/setup.bash`` after running ``catkin_make``.
