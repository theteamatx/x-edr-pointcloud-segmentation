load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# GTest
http_archive(
    name = "com_google_googletest",
    sha256 = "ad7fdba11ea011c1d925b3289cf4af2c66a352e18d4c7264392fead75e919363",
    strip_prefix = "googletest-1.13.0",
    url = "https://github.com/google/googletest/archive/refs/tags/v1.13.0.tar.gz",
)

# Google Benchmark library, used in micro-benchmarks.
http_archive(
    name = "com_google_benchmark",
    sha256 = "1ba14374fddcd9623f126b1a60945e4deac4cdc4fb25a5f25e7f779e36f2db52",
    strip_prefix = "benchmark-d2a8a4ee41b923876c034afb939c4fc03598e622",
    urls = ["https://github.com/google/benchmark/archive/d2a8a4ee41b923876c034afb939c4fc03598e622.zip"],
)

# Absl
abseil_ref = "refs/tags"
abseil_ver = "20230125.0"
http_archive(
    name = "com_google_absl",
    sha256 = "3ea49a7d97421b88a8c48a0de16c16048e17725c7ec0f1d3ea2683a2a75adc21",
    strip_prefix = "abseil-cpp-%s" % abseil_ver,
    url = "https://github.com/abseil/abseil-cpp/archive/%s/%s.tar.gz" % (abseil_ref, abseil_ver),
)


# Protobuf
http_archive(
    name = "com_google_protobuf",
    strip_prefix = "protobuf-main",
    urls = ["https://github.com/protocolbuffers/protobuf/archive/main.zip"],
)

load("@com_google_protobuf//:protobuf_deps.bzl", "protobuf_deps")
protobuf_deps()

http_archive(
    name = "eigen",
    build_file = "//third_party:eigen.BUILD",
    url = "https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz",
    strip_prefix="eigen-3.4.0",
)

local_repository(
    name = "eigenmath",
    path = "third_party/eigenmath",
)

local_repository(
    name = "genit",
    path = "third_party/genit",
)

local_repository(
    name = "mobility_diff_drive",
    path = "third_party/mobility-diff-drive",
)

local_repository(
    name = "mobility_collision",
    path = "third_party/mobility-collision",
)
