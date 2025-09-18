#!/usr/bin/env python3
import typer
from typing import Optional
from enum import Enum
import os
from pathlib import Path
import sys
import cppyy


def _find_workspace_root(start: Path) -> Path:
    p = start.resolve()
    for _ in range(8):
        if (p / "build").is_dir() and (p / "src").is_dir():
            return p
        p = p.parent
    return start.resolve()


def _setup_cppyy(ws_root: Path) -> None:
    # Headers
    include_candidates = [
        ws_root / "src/cloudini/cloudini_lib/tools/include",
        ws_root / "src/cloudini/cloudini_lib/include",
        ws_root / "install/cloudini_lib/include",
    ]
    for inc in include_candidates:
        if inc.is_dir():
            cppyy.add_include_path(str(inc))

    # Libraries (ensure primary deps come first)
    lib_candidates = [
        ws_root / "build/cloudini_lib/libcloudini_lib.so",
        ws_root / "build/cloudini_lib/tools/libmcap_converter.so",
    ]
    for lib in lib_candidates:
        if lib.is_file():
            cppyy.load_library(str(lib))

    # Ensure optional is known for constructing std::optional<float>
    cppyy.cppdef("#include <optional>")

    # Bring converter + Cloudini types into scope
    cppyy.include("mcap_converter.hpp")
    cppyy.include("cloudini_lib/cloudini.hpp")


app = typer.Typer(add_completion=False)


class CompressionMethod(str, Enum):
    none = "none"
    lz4 = "lz4"
    zstd = "zstd"


@app.command()
def convert(
    filename: str = typer.Option(..., "-f", "--filename", help="Input .mcap file"),
    output: Optional[str] = typer.Option(None, "-o", "--output", help="Output .mcap file (auto-named if omitted)"),
    resolution: float = typer.Option(0.001, "-r", "--resolution", help="Resolution for lossy fields (encode only)"),
    profile: Optional[str] = typer.Option(None, "--profile", help="Profile string or path for field resolutions (encode only)"),
    compress: bool = typer.Option(False, "-c", "--compress", help="Convert PointCloud2 to CompressedPointCloud2"),
    decode: bool = typer.Option(False, "-d", "--decode", help="Convert CompressedPointCloud2 to PointCloud2"),
    method: CompressionMethod = typer.Option(CompressionMethod.zstd, "-m", "--method", help="Compression for MCAP chunks when writing"),
    yes: bool = typer.Option(False, "-y", "--yes", help="Overwrite output if it exists"),
) -> None:
    in_path = Path(filename)
    if in_path.suffix != ".mcap":
        typer.echo(f"Error: input must be a .mcap file: {in_path}", err=True)
        raise typer.Exit(code=1)
    if not in_path.is_file():
        typer.echo(f"Error: file not found: {in_path}", err=True)
        raise typer.Exit(code=1)

    if compress and decode:
        typer.echo("Error: cannot specify both --compress and --decode", err=True)
        raise typer.Exit(code=1)
    if not compress and not decode:
        typer.echo("Error: must specify either --compress or --decode", err=True)
        raise typer.Exit(code=1)
    if decode and profile:
        typer.echo("Error: --profile is only valid with --compress", err=True)
        raise typer.Exit(code=1)

    # Resolve workspace root and set up cppyy
    ws_root = _find_workspace_root(Path(__file__).parent)
    _setup_cppyy(ws_root)

    # Compression method mapping
    CompressionOption = cppyy.gbl.Cloudini.CompressionOption
    method_map = {
        "none": CompressionOption.NONE,
        "lz4": CompressionOption.LZ4,
        "zstd": CompressionOption.ZSTD,
    }
    mcap_writer_compression = method_map[method.value]

    # Output filename
    if output:
        out_path = Path(output)
        if out_path.suffix != ".mcap":
            out_path = out_path.with_suffix(".mcap")
    else:
        suffix = "_encoded.mcap" if compress else "_decoded.mcap"
        out_path = in_path.with_name(in_path.stem + suffix)

    if out_path.exists() and not yes:
        typer.echo(f"Error: output exists: {out_path} (use -y to overwrite)", err=True)
        raise typer.Exit(code=1)

    # Run conversion
    converter = cppyy.gbl.McapConverter()
    converter.open(str(in_path))

    if compress:
        # Optional profile
        if profile:
            prof = profile
            p = Path(prof)
            if p.is_file():
                prof = p.read_text().strip()
            converter.addProfile(prof)
        # std::optional<float>(resolution)
        std_optional_float = cppyy.gbl.std.optional['float']
        default_res = std_optional_float(float(resolution))
        converter.encodePointClouds(str(out_path), default_res, mcap_writer_compression)
    else:
        converter.decodePointClouds(str(out_path), mcap_writer_compression)

    typer.echo(str(out_path))


if __name__ == "__main__":
    app()

# ./3_cloudini_cli.py -o testout.mcap -r 0.1 -f ../pointcloud_lexus3-2024-04-05-gyor.mcap --compress -y
# Original size: 541MB
# Compressed   : 289MB