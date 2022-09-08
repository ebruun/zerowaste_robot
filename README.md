# zerowaste_robot

the project was completed with compas_fab 0.25.0, from the ziv261_cfab250 environemt

On September 8th, 2022 (after completion of project) the visualizer component was updated: compas_fab 0.27.0 was installed from pre-compiled (unreleased) version, linked to rhino

Make sure to unlink and relink old version if needed in future...

Excerpt from slack convo:

* "the easiest option is to create an environment as usual, install compas_fab as usual from conda, and then clone the repo and run pip install -e . on the root folder of the repo which will replace the conda install with the source install after that, run python -m compas_rhino.install (as usual)"
* "let me send you a pre-compiled compas_fab from source + components" (this is so components get updated in GH)
* "So, here's a pre-release 0.27 compas_fab"
* "download it and then from the command line, with the conda environment conda_build_update activated, and then install with pip install compas_fab-0.27.0.tar.gz"

This was to fix an issue with ACM not showing up for robot2, wasn't fixed, but this was patched in a live session with Gonzalo

Issue was made on github about this:

https://github.com/compas-dev/compas_fab/issues/372#issue-1366740799