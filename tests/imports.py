#!/usr/bin/env python
"""Test imports from the imitation_learning library."""
import imitation_learning
from imitation_learning.expert import Expert
from imitation_learning.environment import Environment
from imitation_learning.dagger import dagger


if __name__ == '__main__':
    print 'Module name: {}'.format(imitation_learning.__name__)
    print 'Module paths: {}'.format(imitation_learning.__path__)
    print 'Package: {}'.format(imitation_learning.__package__)
    print 'File: {}'.format(imitation_learning.__file__)
    print
    print 'Expert: {}'.format(Expert)
    print 'Environment: {}'.format(Environment)
    print 'Dagger: {}'.format(dagger)

