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
abseil_ver = "20230125.3"
http_archive(
    name = "com_google_absl",
    sha256 = "5366d7e7fa7ba0d915014d387b66d0d002c03236448e1ba9ef98122c13b35c36",
    strip_prefix = "abseil-cpp-%s" % abseil_ver,
    url = "https://github.com/abseil/abseil-cpp/archive/%s/%s.tar.gz" % (abseil_ref, abseil_ver),
)

# Protobuf
_PROTOBUF_VERSION = "24.3"
http_archive(
    name = "com_google_protobuf",
    sha256 = "07d69502e58248927b58c7d7e7424135272ba5b2852a753ab6b67e62d2d29355",
    strip_prefix = "protobuf-%s" % _PROTOBUF_VERSION,
    urls = [
        "https://github.com/protocolbuffers/protobuf/archive/v%s.tar.gz" % _PROTOBUF_VERSION,
    ],
)

load("@com_google_protobuf//:protobuf_deps.bzl", "protobuf_deps")
protobuf_deps()

http_archive(
    name = "eigen",
    build_file = "//third_party:eigen.BUILD",
    sha256 = "8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72",
    url = "https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz",
    strip_prefix="eigen-3.4.0",
)

# GenIt
_GENIT_VERSION = "1.0.0"
http_archive(
    name = "x_edr_genit",
    sha256 = "ab1bbb15ecbe86c5c3888a12c56fe88fac416f2f305acaf1bbf7f68c3d429851",
    strip_prefix = "x-edr-genit-%s" % _GENIT_VERSION,
    urls = [
        "https://github.com/theteamatx/x-edr-genit/archive/refs/tags/v%s.tar.gz" % _GENIT_VERSION,
    ],
)

# Eigenmath
_EIGENMATH_VERSION = "1.0.0"
http_archive(
    name = "x_edr_eigenmath",
    sha256 = "180bf186214b37190e3f26204a271d214b503b25bd22d4228d8f32e7c7151e05",
    strip_prefix = "x-edr-eigenmath-%s" % _EIGENMATH_VERSION,
    urls = [
        "https://github.com/theteamatx/x-edr-eigenmath/archive/refs/tags/v%s.tar.gz" % _EIGENMATH_VERSION,
    ],
)

# Mobility diff-drive
_DIFF_DRIVE_VERSION = "1.0.0"
http_archive(
    name = "x_edr_mobility_diff_drive",
    sha256 = "d4de2bd4aeb17223be3eb6f682fd083f8695f669c277638a7ad75605c81d4ce7",
    strip_prefix = "x-edr-mobility-diff-drive-%s" % _DIFF_DRIVE_VERSION,
    urls = [
        "https://github.com/theteamatx/x-edr-mobility-diff-drive/archive/refs/tags/v%s.tar.gz" % _DIFF_DRIVE_VERSION,
    ],
)

# Mobility diff-drive
_COLLISION_VERSION = "1.0.0"
http_archive(
    name = "x_edr_mobility_collision",
    sha256 = "1d454f9fe387e0cbda8509ee582c1cac68c2ec31887d71b31ca62a38c8266731",
    strip_prefix = "x-edr-mobility-collision-%s" % _COLLISION_VERSION,
    urls = [
        "https://github.com/theteamatx/x-edr-mobility-collision/archive/refs/tags/v%s.tar.gz" % _COLLISION_VERSION,
    ],
)
