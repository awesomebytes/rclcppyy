"""
Thin wrapper exposing C++ rosbag2 APIs through cppyy for Python use.

This module provides access to the common C++ rosbag2 classes and
simple helpers to open readers/writers and iterate messages.

Usage:
    from rclcppyy import rosbag2_cpp as rosbag2_py
    reader = rosbag2_py.open_reader("/path/to/input", storage_id="mcap")
    writer = rosbag2_py.open_writer("/path/to/output", storage_id="mcap")
"""

import os
import glob
import sys
import ctypes
from typing import Iterable, Tuple, Dict, Any, Optional, List, Set

import cppyy
from ament_index_python.packages import get_packages_with_prefixes, get_package_prefix

from rclcppyy.bringup_rclcpp import bringup_rclcpp


_BROUGHT_UP: bool = False
_LOADED_LIBS: Set[str] = set()


def _load_library_once(so_path: str) -> None:
    if not so_path or not os.path.exists(so_path):
        return
    if so_path in _LOADED_LIBS:
        return
    try:
        # Ensure global symbol visibility for plugin resolution
        ctypes.CDLL(so_path, mode=ctypes.RTLD_GLOBAL)
    except Exception:
        pass
    try:
        cppyy.load_library(so_path)
    except Exception:
        # cppyy may fail to load some libs that are not strictly needed; ignore
        pass
    _LOADED_LIBS.add(so_path)


def _glob_and_load(lib_dir: str, patterns: List[str]) -> None:
    if not os.path.isdir(lib_dir):
        return
    for pat in patterns:
        # Try naked, .so and versioned .so.*
        for glob_pat in [pat, pat + '.so', pat + '.so.*']:
            for so_path in sorted(set(glob.glob(os.path.join(lib_dir, glob_pat)))):
                _load_library_once(so_path)


def _preload_rosbag2_libraries() -> None:
    """
    Load core rosbag2 and dependency libraries from their package prefixes,
    in a stable order, avoiding wide scans across all prefixes.
    """
    pkg_load_order: List[List[str]] = [
        ['rcutils'],
        ['rcpputils'],
        ['console_bridge'],
        ['tinyxml2', 'tinyxml2_vendor'],
        ['yaml-cpp', 'yaml_cpp_vendor'],
        ['class_loader'],
        ['pluginlib'],
        ['rosbag2_storage'],
        ['rosbag2_cpp'],
        ['rosbag2_cpp_serialization'],  # may not exist on all distros
    ]

    pkg_to_patterns: Dict[str, List[str]] = {
        'rcutils': ['librcutils*'],
        'rcpputils': ['librcpputils*'],
        'console_bridge': ['libconsole_bridge*'],
        'tinyxml2': ['libtinyxml2*'],
        'tinyxml2_vendor': ['libtinyxml2*'],
        'yaml-cpp': ['libyaml-cpp*'],
        'yaml_cpp_vendor': ['libyaml-cpp*'],
        'class_loader': ['libclass_loader*'],
        'pluginlib': ['libpluginlib*'],
        'rosbag2_storage': ['librosbag2_storage*'],
        'rosbag2_cpp': ['librosbag2_cpp*'],
        'rosbag2_cpp_serialization': ['librosbag2_cpp_serialization*'],
    }

    for pkg_group in pkg_load_order:
        loaded_any = False
        for pkg in pkg_group:
            try:
                prefix = get_package_prefix(pkg)
            except Exception:
                continue
            lib_dir = os.path.join(prefix, 'lib')
            patterns = pkg_to_patterns.get(pkg, [])
            if patterns:
                _glob_and_load(lib_dir, patterns)
                loaded_any = True
        # Continue regardless of whether group was found; some groups are optional

    # Optionally, load rosbag2_py extensions (can carry symbols) from all known prefixes
    try:
        for _, prefix in get_packages_with_prefixes().items():
            site_pkgs = os.path.join(prefix, 'lib', f'python{sys.version_info.major}.{sys.version_info.minor}', 'site-packages')
            rosbag2_py_dir = os.path.join(site_pkgs, 'rosbag2_py')
            if os.path.isdir(rosbag2_py_dir):
                for so_path in sorted(glob.glob(os.path.join(rosbag2_py_dir, '*.so'))):
                    _load_library_once(so_path)
    except Exception:
        pass


def _ensure_storage_plugin_loaded(storage_id: str) -> None:
    """
    Best-effort load of the storage plugin matching the given storage_id (e.g., 'mcap', 'sqlite3').
    Scans package lib directories for likely plugin names and loads them.
    """
    if not storage_id:
        return
    try:
        prefixes = list(set(get_packages_with_prefixes().values()))
    except Exception:
        prefixes = []
    # Also include commonly relevant prefixes directly if resolvable
    for pkg in ['rosbag2_storage', f'rosbag2_storage_{storage_id}', f'rosbag2_{storage_id}']:
        try:
            prefixes.append(get_package_prefix(pkg))
        except Exception:
            pass

    patterns = [
        f'librosbag2_storage*{storage_id}*',
        f'lib*{storage_id}*plugin*',
        f'lib*{storage_id}*',
    ]
    for prefix in sorted(set(prefixes)):
        lib_dir = os.path.join(prefix, 'lib')
        _glob_and_load(lib_dir, patterns)


def _add_minimal_include_paths() -> None:
    for pkg in ['rosbag2_cpp', 'rosbag2_storage', 'rclcpp']:
        try:
            cppyy.add_include_path(os.path.join(get_package_prefix(pkg), 'include'))
        except Exception as e:
            print(f"Error adding include path for {pkg}: {e}")


def bringup_rosbag2_cpp() -> Any:
    """
    Ensure rosbag2_cpp C++ symbols are available through cppyy.
    Returns the cppyy.gbl.rosbag2_cpp namespace.
    """
    global _BROUGHT_UP
    if _BROUGHT_UP:
        return cppyy.gbl.rosbag2_cpp

    bringup_rclcpp()

    # Preload libraries and add minimal include paths
    _preload_rosbag2_libraries()
    _add_minimal_include_paths()

    # Headers (rosbag2 + concrete sequential impls for factories)
    cppyy.include('rosbag2_cpp/reader.hpp')
    cppyy.include('rosbag2_cpp/writer.hpp')
    cppyy.include('rosbag2_cpp/converter_options.hpp')
    cppyy.include('rosbag2_cpp/serialization_format_converter_factory.hpp')
    cppyy.include('rosbag2_cpp/readers/sequential_reader.hpp')
    cppyy.include('rosbag2_cpp/writers/sequential_writer.hpp')
    cppyy.include('rosbag2_storage/storage_options.hpp')
    cppyy.include('rosbag2_storage/serialized_bag_message.hpp')
    cppyy.include('rosbag2_storage/topic_metadata.hpp')
    cppyy.include('rosbag2_storage/storage_factory.hpp')
    cppyy.include('rosbag2_storage/metadata_io.hpp')
    # Optional convenience type
    try:
        cppyy.include('rclcpp/serialized_message.hpp')
    except Exception:
        pass

    # Factory helpers to avoid unique_ptr copy semantics in constructor binding
    cppyy.cppdef(r"""
#include <memory>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_cpp/serialization_format_converter_factory.hpp>
#include <rosbag2_storage/storage_factory.hpp>
#include <rosbag2_storage/metadata_io.hpp>
extern "C" rosbag2_cpp::Reader* rclcppyy_make_reader_default() {
    auto storage_factory = std::make_unique<rosbag2_storage::StorageFactory>();
    auto converter_factory = std::make_shared<rosbag2_cpp::SerializationFormatConverterFactory>();
    auto metadata_io = std::make_unique<rosbag2_storage::MetadataIo>();
    auto seq_reader = std::make_unique<rosbag2_cpp::readers::SequentialReader>(
        std::move(storage_factory), converter_factory, std::move(metadata_io));
    return new rosbag2_cpp::Reader(std::move(seq_reader));
}
extern "C" rosbag2_cpp::Writer* rclcppyy_make_writer_default() {
    auto storage_factory = std::make_unique<rosbag2_storage::StorageFactory>();
    auto converter_factory = std::make_shared<rosbag2_cpp::SerializationFormatConverterFactory>();
    auto metadata_io = std::make_unique<rosbag2_storage::MetadataIo>();
    auto seq_writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>(
        std::move(storage_factory), converter_factory, std::move(metadata_io));
    return new rosbag2_cpp::Writer(std::move(seq_writer));
}
""")

    _BROUGHT_UP = True
    return cppyy.gbl.rosbag2_cpp


def open_reader(uri: str, *, storage_id: str = 'mcap', serialization: str = 'cdr') -> Any:
    """
    Create and open a C++ rosbag2_cpp::Reader.
    """
    bringup_rosbag2_cpp()
    _ensure_storage_plugin_loaded(storage_id)
    ReaderPtrFactory = cppyy.gbl.rclcppyy_make_reader_default
    StorageOptions = cppyy.gbl.rosbag2_storage.StorageOptions
    ConverterOptions = cppyy.gbl.rosbag2_cpp.ConverterOptions

    reader = ReaderPtrFactory()
    inopt = StorageOptions()
    inopt.uri = uri
    inopt.storage_id = storage_id

    conv = ConverterOptions()
    conv.input_serialization_format = serialization
    conv.output_serialization_format = serialization

    # Open with full options
    reader.open(inopt, conv)
    return reader


def open_writer(uri: str, *, storage_id: str = 'mcap', serialization: str = 'cdr') -> Any:
    """
    Create and open a C++ rosbag2_cpp::Writer.
    """
    bringup_rosbag2_cpp()
    _ensure_storage_plugin_loaded(storage_id)
    WriterPtrFactory = cppyy.gbl.rclcppyy_make_writer_default
    StorageOptions = cppyy.gbl.rosbag2_storage.StorageOptions
    ConverterOptions = cppyy.gbl.rosbag2_cpp.ConverterOptions

    writer = WriterPtrFactory()
    outopt = StorageOptions()
    outopt.uri = uri
    outopt.storage_id = storage_id

    conv = ConverterOptions()
    conv.input_serialization_format = serialization
    conv.output_serialization_format = serialization

    writer.open(outopt, conv)
    return writer


def create_topic(writer: Any, name: str, type_: str, *, serialization_format: str = 'cdr', offered_qos_profiles: str = '') -> None:
    """
    Create a topic on a C++ rosbag2_cpp::Writer.
    """
    bringup_rosbag2_cpp()
    TopicMetadata = cppyy.gbl.rosbag2_storage.TopicMetadata
    md = TopicMetadata()
    md.name = name
    md.type = type_
    md.serialization_format = serialization_format
    # offered_qos_profiles may not exist on all distros; wrap defensively
    try:
        md.offered_qos_profiles = offered_qos_profiles
    except Exception:
        pass
    writer.create_topic(md)


def iter_topics(reader: Any) -> Iterable[Any]:
    """
    Yield topic metadata objects from a C++ reader.
    """
    bringup_rosbag2_cpp()
    try:
        topics = reader.get_all_topics_and_types()
    except Exception:
        topics = reader.get_topics_and_types()
    # Return as-is (vector of TopicMetadata)
    for md in topics:
        yield md


def iter_messages(reader: Any) -> Iterable[Any]:
    """
    Yield C++ rosbag2_storage::SerializedBagMessage shared_ptrs from a reader.
    """
    while reader.has_next():
        yield reader.read_next()


def write_serialized_message(writer: Any, sbm_shared_ptr: Any) -> None:
    """
    Write a C++ rosbag2_storage::SerializedBagMessage shared_ptr to the writer.
    """
    writer.write(sbm_shared_ptr)


# Convenience shortcuts to C++ types for advanced users
def types() -> Dict[str, Any]:
    bringup_rosbag2_cpp()
    t: Dict[str, Any] = {
        'Reader': cppyy.gbl.rosbag2_cpp.Reader,
        'Writer': cppyy.gbl.rosbag2_cpp.Writer,
        'StorageOptions': cppyy.gbl.rosbag2_storage.StorageOptions,
        'ConverterOptions': cppyy.gbl.rosbag2_cpp.ConverterOptions,
        'TopicMetadata': cppyy.gbl.rosbag2_storage.TopicMetadata,
        'SerializedBagMessage': cppyy.gbl.rosbag2_storage.SerializedBagMessage,
    }
    try:
        t['SerializedMessage'] = cppyy.gbl.rclcpp.SerializedMessage
    except Exception:
        pass
    return t


