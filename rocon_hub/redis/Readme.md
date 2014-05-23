There are changes from version to version of the redis server. We typically sniff the version
and apply an ubuntu redis.conf file with local configuration file overrides.

## Hacks

See the .local files for comments on our particular over-rides. There is one tweak to a copied
ubuntu configuration that we need though because non-root calling of the redis-server will throw an error.

* logfile : comment out

A second modification is to point to our local file overrides (at the bottom of the conf file). The rocon_hub
script will fill in this value before launching the redis server.

* include %(local_conf_filename)s

### 2.6/2.8

Delete a comment in redis-2.6.conf/redis-2.8.conf so python substitution can work..

* 10% -> 10 per cent

