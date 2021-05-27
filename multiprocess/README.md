Multiprocess Drake Diagrams
===========================

This directory contains tools to build deterministic diagrams that cross
process boundaries.  This is an experimental feature under development.

Assume that you have two diagrams that for whatever reason (computational
parallelism, dependency issues, data safety, etc.) you want to run in
different processes but which exchange data as if they were in a single
simulation.

LCM-over-memq cannot cross process boundaries and LCM-over-mudp is inherently
unreliable; moreover for message delivery order to be deterministic the
transport layer must control the message envelope.  So this multiprocess
system has a distinct transport wherein a central message dispatch unit has
TCP connections to each individual diagram process and the LCM is tunneled
over that substrate.


Architecture
------------

We want a "Central Metasimulator" compoment that presents a simulator-like API
but has no simulator itself.  This metasim consists of two parts:

* a "server" that is callable and which owns the message dispatch.
* a "client" that owns individual process simulators and message transport.

The metasim client is a builder-complementary object that is created in an
incomplete state.  While in the incomplete state it can add transport systems
to a diagram.  When told to complete, it can no longer add transport, but it
creates and owns a simulator.

The metasim server is similarly incomplete for some period of time.  Prior to
simulation start it allows clients to connect to it.  Once the simulation
starts, clients may no longer connect.


Message Semantics
-----------------

* `SEND_MESSAGE` structure: A sent message with an envelope declaring the time
  within the client simulator when the message was handed off to the client.
* `QUEUED_MESSAGE` structure: A `SEND_MESSAGE` augmented with a delivery time.
* `DELIVERED_MESSAGE` structure: Identical to `QUEUED_MESSAGE` but sent from
  server to client.
* `AUTHORIZE_ADVANCE` structure: Contains a time to which that client is
  permitted to advance its simulation clock.  The client may unconditionally
  advance to that time and then continue to advance until it next polls for
  messages.
* `PROMISE_NO_SEND` structure: Certifies by a client that it will not send
  any further messages prior to a particular time.
* `NOTIFY_ADVANCE` structure: Certifies by a client that it has advanced to a
  particular time; this must contain a `PROMISE_NO_SEND`.
* `POLL_WAIT` structure: Indicates that a client has reached a particular time
  and will block, accumulating messages, until authorized to advance.
  Identical in content to `NOTIFY_ADVANCE`.


Runtime Procedure
-----------------

### Process Start

As with any other simulator, the metasim server waits to be authorized to
advance time.  When it is authorized to advance time, the exciting things
start.

The first thing that happens in a simulation is the "epsilon tick".  This is
the collection of "dawn of time" messages that most senders emit.  Each of
these messages is stamped with an origin time by its client and delivered to
the server.  The server then computes a delivery time for each message to each
client, and enrolls the message in a delivery queue for each recipient.

(Note that this means that delivery time computation can consider both the
sender and recipient if you want to model heterogeneous and asymmetric delays.
Packet loss can also be introduced at this stage.)

(NOTE:  For soundness there *must* be a minimum delivery time delay.)

### Steady State

We now enter the steady state operation of the simulation.

* Loop:
  * The safe advance time is the earliest promise-no-send time of any process
    plus the minimum delivery delay.
  * Iterate over the clients from earliest time to latest.
    * If this client is in poll wait
      * Deliver all queued messages for that client earlier than the safe
        advance time.
      * Authorize it to advance to the safe advance time.
  * Handle incoming messages, queuing `SEND_MESSAGE` messages as described
    above, and otherwise updating the current and no-send times of each
    process as they are announced.

### Notes

* Deadlock-free guarantee:
  * The minimum delivery delay guarantees that the safe advance time always lies
    in the future of at least once process; if that process is in poll state
    then it may be authorized; if not then it is running.
  * Ergo there is always either a running process or a process that can be
    authorized.
* Parallelism in practice:
  * With zero minimum delivery delay and with no use of `PROMISE_NO_SEND` by
    clients, there can sometimes be zero authorized processes.
  * For a small minimum delivery delay and with no use of `PROMISE_NO_SEND` by
    clients, only one process will ever be authorized to advance; this is
    in effect a global mutex.
  * The number of processes that can be authorized to run in parallel depends
    on the delivery delay and the use of `PROMISE_NO_SEND`.  Increasing either
    one increases parallelism.
* Error conditions:
  * The `System` interface allows a force publish at any time, which implicitly
    violates any possible `PROMISE_NO_SEND` calculation.  It will be necessary
    for each client to track its own promises and emit an error in this case.
    * The server could detect this, but local detection is more sound.


Implementation
--------------

The metasim server can in theory be written either in python or in C++.  It
will initially be written in C++ so that it can easily and
more-or-less-soundly subclass `Simulator`.

Metasimulation will be tested only for `<T=double>` for now.

The client should certainly be in C++ because it will need to have nontrivial
ownership semantics over network resources like LCMs.

There is going to be complexity around the message substrate.  I have assumed
a completely generic messaging substrate, but using a real one like LCM will
be more complex because, eg, "channel" semantics in a multicast message system
create a point of conflict between multiple instances of the same software.

To manage this, we declare:
 * The implementation will tunnel LCM over TCP in order to guarantee
   reliable, in-order messaging.
   * There are two existing lcm-over-tcp systems (lcm tcpq and
     bot-lcm-tunnel), but both are GPL-encumbered and provide slightly wrong
     featuresets that prevent use as-is.  So the wheel will be herein
     reinvented.
 * The metaserver will deliver every received and delivered message to a
   public LCM.
 * The public LCM will have channel names that have a client identifier
   appended to the declare channel name.
 * The public LCM messages will carry the enclosure timestamp (ie the
   `time_sec` passed in the underlying call to `drake::lcm::Publish`).

