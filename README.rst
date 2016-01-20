======
mc2lib
======

.. image:: https://travis-ci.org/melver/mc2lib.svg?branch=master
    :target: https://travis-ci.org/melver/mc2lib

A memory consistency model checking library implemented in C++11. This library
provides the building blocks for the `McVerSi
<http://ac.marcoelver.com/research/mcversi>`_ framework [1]_.

The McVerSi guest workload can be found in ``contrib/mcversi``.

Usage
=====

The library is header-only and no compilation is required.

API documentation can be found `here
<http://ac.marcoelver.com/ext/apidoc/mc2lib>`_.

Citation
========

If you use this library or the McVerSi framework in your work, we would
appreciate if you cite the McVerSi paper:

.. code-block::

    @inproceedings{ElverN2016,
      author    = {Marco Elver and Vijay Nagarajan},
      title     = {{McVerSi}: {A} {T}est {G}eneration {F}ramework for {F}ast
                   {M}emory {C}onsistency {V}erification in {S}imulation},
      booktitle = {IEEE International Symposium on
                   High Performance Computer Architecture (HPCA)},
      month     = mar,
      year      = {2016},
      venue     = {Barcelona, Spain}
    }

References
==========

.. [1] Marco Elver and Vijay Nagarajan. `McVerSi: A Test Generation Framework
       for Fast Memory Consistency Verification in Simulation
       <http://ac.marcoelver.com/res/hpca2016-mcversi.pdf>`_. In IEEE
       International Symposium on High Performance Computer Architecture
       (HPCA). Barcelona, Spain, March 2016.
