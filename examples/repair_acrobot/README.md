Example: Automatic repair of acrobot Spong controller
-----------------------------------------------------

## Overview

This example is drawn from TRI's work on automated controller repair, in which
simulations over a distribution of initial states are used to update the
parameters of a controller to improve its performance.

Repair requires:
 * A gradient-free optimizer
 * A simulation
 * A distribution of initial states
 * A metric to minimize
 * Some decision variables

In this example we use Facebook's Nevergrad optimizer over an acrobot with a
swing-up controller; it is tested over a random sample from all possible
initial conditions, optimizing swing-up time by manipulating the controller's
four parameters.

Because repair requires running a potentially large number of evaluations we
provide a general interface for systems to repeatedly run a bazel target with
different inputs.  We provide interfaces to run locally or on a cluster of
remote hosts via ssh; it is also possible to use the cloud for this but we
must leave that as an exercise for the reader as our cloud backend is specific
to TRI's infrastructure.

## How to run the example

 * You will need to install Facebook's "Nevergrad" optimizer https://github.com/facebookresearch/nevergrad

   $ pip3 install nevergrad

 * Run `run_spong_controller_repair` to demonstrate finding good
   parameters.

   $ bazel run //examples/repair_acrobot:run_spong_controller_repair \
     --scenario examples/repair_acrobot/test/sample_scenario.yaml \
     --output repair_result.yaml

 * The output yaml will give a good set of parameters for a spong controller
   on the sample acrobot.

## Repair Problem Details

### Optimizer

xxx

### Simulation

The core simulation is provided by `spong_sim.cc`, which just wraps
`drake/examples/acrobot` with yaml reading (to read initial conditions from a
file) and writing (to write state recordings to a file).

### Distributions

xxx

### Metrics

xxx

### Decision Variables

xxx

## Runtime Details

### `JobRunnerBase`

xxx

### `LocalJobRunner`

xxx

### `RemoteJobRunner`

xxx
