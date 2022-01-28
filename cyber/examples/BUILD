load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "talker",
    srcs = ["talker.cc"],
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
    ],
)

cc_binary(
    name = "listener",
    srcs = ["listener.cc"],
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
    ],
)

install(
    name = "install",
    targets = [
        ":talker",
        ":listener",
    ],
    runtime_dest = "cyber/examples",
    deps = [
        "//cyber/examples/common_component_example:install",
        "//cyber/examples/timer_component_example:install",
    ],
)

cc_binary(
    name = "paramserver",
    srcs = ["paramserver.cc"],
    deps = [
        "//cyber",
        "//cyber/parameter",
    ],
)

cc_binary(
    name = "service",
    srcs = ["service.cc"],
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
    ],
)

cc_binary(
    name = "record",
    srcs = ["record.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:record_cc_proto",
    ],
)


cc_binary(
    name = "udp_writer",
    srcs = [
        "udp_writer.cc",
        "DetectionData.h",
        "ChatterData.h",
        "microbuf.h",
        ],
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
        "//modules/perception/proto:perception_obstacle_cc_proto",
        "@boost",
    ],
)

cpplint()
