const std = @import("std");

const fnctl = @cImport(@cInclude("fcntl.h"));
const ioctl = @cImport(@cInclude("sys/ioctl.h")).ioctl;
const time = @cImport(@cInclude("sys/time.h"));
const unistd = @cImport(@cInclude("unistd.h"));

const v4l2 = @cImport(@cInclude("linux/videodev2.h"));
const libyuv = @cImport(@cInclude("libyuv.h"));

const zargs = @import("args");

// If you're unsure about your video4linux2 setup, try this to capture a frame from the webcam:
// v4l2-ctl --device /dev/video0 --set-fmt-video=width=1280,height=720,pixelformat=YUYV --stream-mmap --stream-to=frame.raw --stream-count=1

// TDPF_2007_art00005_Sergey-Bezryadin.pdf
// Convert RGB to stimulus length

const PROGRAM_NAME = "backlight-auto";

pub fn main() !void {
    var GPA = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = GPA.deinit();
    const allocator = GPA.allocator();

    const Options = struct {
        measure: bool = false,
        @"sample-time": u8 = 4,
        @"min-stimulus-length": ?f32 = null,
        @"path-dev-video": []const u8 = "/dev/video0",
        @"path-backlight": []const u8 = "/sys/class/backlight/intel_backlight",
        help: bool = false,

        pub const shorthands = .{
            .m = "measure",
            .s = "sample-time",
            .l = "min-stimulus-length",
            .v = "path-dev-video",
            .b = "path-backlight",
            .h = "help",
        };

        pub const meta = .{
            .option_docs = .{
                .measure = "Measure the \"darkest stimulus\" length of the webcam. Make sure the webcam is covered!",
                .@"sample-time" = "Brightness sampling time (in seconds). Default is 4.",
                .@"min-stimulus-length" = "Set the stimulus length calculated with `--measure`.",
                .@"path-dev-video" = "Path to the video device (eg. /dev/video0).",
                .@"path-backlight" = "Path to system backligh directory (eg. /sys/class/backlight/intel_backlight).",
                .help = "Display this help and exit.",
            },
        };
    };

    const arguments = try zargs.parseForCurrentProcess(Options, allocator, .print);
    defer arguments.deinit();

    const stdout = std.io.getStdOut().writer();

    if (arguments.options.help) return zargs.printHelp(Options, arguments.executable_name orelse "backlight-auto", stdout);

    if (arguments.options.measure) {
        const pixels = try readWebcamPixels(
            allocator,
            arguments.options.@"path-dev-video",
            arguments.options.@"sample-time",
        );
        defer allocator.free(pixels);

        const avg_stim_len = averageStimulusLength(pixels);

        try stdout.print("The minimal stimulus length of {s} is: {}\n", .{ arguments.options.@"path-dev-video", avg_stim_len });
        return;
    } else {
        if (arguments.options.@"min-stimulus-length") |min_stimulus_length| {
            const pixels = try readWebcamPixels(
                allocator,
                arguments.options.@"path-dev-video",
                arguments.options.@"sample-time",
            );
            defer allocator.free(pixels);

            const avg_stim_len = averageStimulusLength(pixels);

            const backlight_max_brightness = try readBacklightFile(
                allocator,
                arguments.options.@"path-backlight",
            );

            const brightness = calculateNewBrightness(avg_stim_len, backlight_max_brightness, min_stimulus_length);
            try stdout.print("{d}", .{brightness});
        } else {
            try stdout.print("Please provide a minimal stimulus length value. You can measure it with '-m' option.\n", .{});
        }
    }
}

const RGB = struct { r: u8, g: u8, b: u8 };
const XYZ = struct { x: f32, y: f32, z: f32 };

fn rgb2xyz(rgb: RGB) XYZ {
    // CIE matrix
    const cie: [3][3]f32 = .{
        .{ 0.49000, 0.31000, 0.20000 },
        .{ 0.17697, 0.81240, 0.01063 },
        .{ 0.00000, 0.01000, 0.99000 },
    };

    return XYZ{
        .x = cie[0][0] * @as(f32, @floatFromInt(rgb.r)) + cie[0][1] *
            @as(f32, @floatFromInt(rgb.g)) + cie[0][2] * @as(f32, @floatFromInt(rgb.b)),
        .y = cie[1][0] * @as(f32, @floatFromInt(rgb.r)) + cie[1][1] *
            @as(f32, @floatFromInt(rgb.g)) + cie[1][2] * @as(f32, @floatFromInt(rgb.b)),
        .z = cie[2][0] * @as(f32, @floatFromInt(rgb.r)) + cie[2][1] *
            @as(f32, @floatFromInt(rgb.g)) + cie[2][2] * @as(f32, @floatFromInt(rgb.b)),
    };
}

fn xyzToStimulusLength(xyz: XYZ) f32 {
    // Cohen metrics
    const cm: [3][3]f32 = .{
        .{ 0.2053, 0.7125, 0.4670 },
        .{ 1.8537, -1.2797, -0.4429 },
        .{ -0.3655, 1.0120, -0.6104 },
    };

    const d = cm[0][0] * xyz.x + cm[0][1] * xyz.y + cm[0][2] * xyz.z;
    const e = cm[1][0] * xyz.x + cm[1][1] * xyz.y + cm[1][2] * xyz.z;
    const f = cm[2][0] * xyz.x + cm[2][1] * xyz.y + cm[2][2] * xyz.z;
    return @sqrt(d * d + e * e + f * f);
}

const DeviceImageBytes = struct {
    start: ?[]align(std.mem.page_size) const u8,
    length: usize,
};

fn deviceImageBytesFromYUVToRGB(
    allocator: std.mem.Allocator,
    dib: DeviceImageBytes,
    format: v4l2.v4l2_format,
) ![]const u8 {
    // How to actually call this, I learned from:
    // libyuv in unit_test/color_test.cc:166-170
    // YUV converted to ARGB.

    const w: c_int = @intCast(format.fmt.pix.width);
    const h: c_int = @intCast(format.fmt.pix.height);

    var pixels = try allocator.alloc(u8, @as(usize, @intCast(w)) * @as(usize, @intCast(h)) * 4);

    _ = libyuv.I422ToARGB(@ptrCast(dib.start), w, @ptrCast(dib.start), @divTrunc((w + 1), 2), @ptrCast(dib.start), @divTrunc((w + 1), 2), @ptrCast(pixels), w * 4, w, h);

    return pixels;
}

fn averageStimulusLength(pixels: []const u8) f32 {
    var avg: f32 = 0;
    var index: usize = 0;

    while (index < pixels.len) : (index += 4) {
        const rgb = .{
            .r = pixels[index + 0],
            .g = pixels[index + 1],
            .b = pixels[index + 2],
        };
        const xyz = rgb2xyz(rgb);
        const sl = xyzToStimulusLength(xyz);
        avg += sl;
    }

    return avg / (@as(f32, @floatFromInt(pixels.len)) / 4.0);
}

fn readBacklightFile(allocator: std.mem.Allocator, path: []const u8) !f32 {
    const path_brightness_max = try std.mem.concat(allocator, u8, &.{
        path,
        "/max_brightness",
    });
    defer allocator.free(path_brightness_max);

    const file = try std.fs.openFileAbsolute(path_brightness_max, .{ .mode = std.fs.File.OpenMode.read_only });
    const file_text = try file.readToEndAllocOptions(allocator, 256, null, 8, '\n');
    defer allocator.free(file_text);

    const backlight: u16 = try std.fmt.parseInt(u16, file_text[0 .. file_text.len - 1], 10);
    return @floatFromInt(backlight);
}

fn calculateNewBrightness(avgsl: f32, max_brightness: f32, min_stimulus_length: f32) u16 {
    return @intFromFloat(
        @max(1.0, max_brightness * (@sqrt(@max(0.0, avgsl - min_stimulus_length)) / 4.0)),
    );
}

fn isSupportedFormat(target: u32) bool {
    const FORMATS_SUPPORTED = [_]u32{
        v4l2.V4L2_PIX_FMT_YUYV,
    };

    for (FORMATS_SUPPORTED) |supported| {
        if (target != supported) continue;
        return true;
    }
    return false;
}

fn readWebcamPixels(allocator: std.mem.Allocator, path: []const u8, sample_time: u8) ![]const u8 {
    const fd = fnctl.open(@ptrCast(path), fnctl.O_RDWR);
    if (fd == -1) {
        std.log.err("fnctl.open failed, check if You passed the proper video device", .{});
        return error.FNCTLOpenError;
    }

    const cap = v4l2.v4l2_capability{
        .driver = .{},
        .card = .{},
        .version = 0,
        .capabilities = 0,
        .device_caps = 0,
        .bus_info = .{},
        .reserved = .{},
    };

    if (ioctl(fd, v4l2.VIDIOC_QUERYCAP, &cap) == -1) {
        return error.SetCapabilityFailed;
    }

    if (cap.capabilities & v4l2.V4L2_CAP_VIDEO_CAPTURE != v4l2.V4L2_CAP_VIDEO_CAPTURE) {
        std.log.err("Video capture not supported", .{});
        return error.VideoCaptureUnsupported;
    }

    if (cap.capabilities & v4l2.V4L2_CAP_STREAMING != v4l2.V4L2_CAP_STREAMING) {
        std.log.err("V4L2_CAP_STREAMING not supported", .{});
        return error.StreamingUnsupported;
    }

    const input = v4l2.v4l2_input{
        .index = 0,
        .name = .{},
        .type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .audioset = 0,
        .tuner = 0,
        .std = 0,
        .status = 0,
        .capabilities = 0,
        .reserved = .{},
    };

    if (ioctl(fd, v4l2.VIDIOC_ENUMINPUT, &input) == -1) {
        std.log.err("Failure to get input", .{});
        return error.InputFailure;
    }

    var fmtdesc = v4l2.v4l2_fmtdesc{
        .index = 0,
        .type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .flags = 0,
        .description = .{},
        .pixelformat = 0,
        .mbus_code = 0,
        .reserved = .{},
    };

    const pixelformat = blk: {
        for (0..std.math.maxInt(u8)) |index| {
            fmtdesc.index = @intCast(index);

            if (ioctl(fd, v4l2.VIDIOC_ENUM_FMT, &fmtdesc) == -1) break;

            if (!isSupportedFormat(fmtdesc.pixelformat)) {
                continue;
            }

            std.log.debug("{s}", .{fmtdesc.description});
            break :blk fmtdesc.pixelformat;
        }
        std.log.err("No supported video formats found", .{});
        return error.UnsupportedVideoFormat;
    };

    const format = v4l2.v4l2_format{
        .type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .fmt = .{ .pix = .{
            .width = 0,
            .height = 0,
            .pixelformat = pixelformat,
            .field = 0,
            .bytesperline = 0,
            .sizeimage = 0,
            .colorspace = 0,
            .priv = 0,
            .flags = 0,
            .unnamed_0 = .{
                .hsv_enc = 0,
            },
            .quantization = 0,
            .xfer_func = 0,
        } },
    };

    if (ioctl(fd, v4l2.VIDIOC_G_FMT, &format) == -1) {
        std.log.err("Failed to get image format", .{});
        return error.GetImageFormatFailed;
    }

    if (ioctl(fd, v4l2.VIDIOC_S_FMT, &format) == -1) {
        std.log.err("Failed to set image format", .{});
        return error.SetImageFormatFailed;
    }

    var buffer_copy = DeviceImageBytes{
        .start = null,
        .length = 0,
    };

    var requestbuffers = v4l2.v4l2_requestbuffers{
        .count = 1,
        .type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = v4l2.V4L2_MEMORY_MMAP,
        .reserved = .{},
        .capabilities = 0,
        .flags = 0,
    };

    if (ioctl(fd, v4l2.VIDIOC_REQBUFS, &requestbuffers) == -1) {
        std.log.err("Failed to request buffers", .{});
        return error.RequestBuffersFailed;
    }

    var buffer = v4l2.v4l2_buffer{
        .index = 0,
        .type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = v4l2.V4L2_MEMORY_MMAP,
        .bytesused = 0,
        .flags = 0,
        .field = 0,
        .timestamp = v4l2.timeval{
            .tv_sec = 0,
            .tv_usec = 0,
        },
        .timecode = v4l2.v4l2_timecode{
            .type = 0,
            .flags = 0,
            .frames = 0,
            .seconds = 0,
            .minutes = 0,
            .hours = 0,
            .userbits = .{},
        },
        .sequence = 0,
        .m = .{
            .offset = 0,
        },
        .length = 0,
        .reserved2 = 0,
        .unnamed_0 = .{
            .reserved = 0,
        },
    };

    if (ioctl(fd, v4l2.VIDIOC_QUERYBUF, &buffer) == -1) {
        std.log.err("Failed to query buffer", .{});
        return error.BufferQueryFailed;
    }

    var seconds: u8 = 0;
    while (seconds < sample_time) : (std.time.sleep(1e9)) {
        seconds += 1;
        _ = ioctl(fd, v4l2.VIDIOC_QBUF, &buffer);
        _ = ioctl(fd, v4l2.VIDIOC_STREAMON, &v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE);
        _ = ioctl(fd, v4l2.VIDIOC_DQBUF, &buffer);
    }

    buffer_copy.length = buffer.length;
    buffer_copy.start = std.os.mmap(
        null,
        buffer.length,
        std.os.linux.PROT.READ | std.os.linux.PROT.WRITE,
        std.os.linux.MAP.SHARED,
        fd,
        buffer.m.offset,
    ) catch {
        std.log.err("Failed to mmap", .{});
        return error.MmapFailed;
    };

    const pixels = switch (pixelformat) {
        v4l2.V4L2_PIX_FMT_YUYV => try deviceImageBytesFromYUVToRGB(allocator, buffer_copy, format),
        else => {
            std.log.err("Unknown pixel format: {d}", .{pixelformat});
            return error.UnknownPixelFormat;
        },
    };

    if (buffer_copy.start) |memory| {
        std.os.munmap(memory);
    }

    return pixels;
}
