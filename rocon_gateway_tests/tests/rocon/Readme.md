## Multimaster Tests

Example usage:

```
> rocon_test rocon_gateway_tests xyz.test
```

When tests are built by ros (on the build farm, or with yujin_make), they are usually done in parallel, so
make sure each test is using a different set of ports.

I usually reserve ten for each test.

