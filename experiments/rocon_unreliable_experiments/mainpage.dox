/**
\mainpage
\htmlinclude manifest.html


@section keyPoints Key Points

When creating a subscriber you can specify hints to make it unreliable (udp)
You cannot do anything on the publisher end
If the publisher (not roscpp publishers) does not support udp, then tcp connections are made.

@section Experimenting

One publisher, one unreliable subscriber - udp connection
One publisher, one subscriber, one unreliable subscriber - the latter is still udp connection
You can observe the unreliable connections being made with:

@code
> watch -n1 ss -u
@endcode

@section Conclusions

- It’s only really important to make sure subscribers are unreliable.
- The publishers can be exposed without any relaying or configuration.


*/
