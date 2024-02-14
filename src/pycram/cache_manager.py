import os
import pathlib
import re

from typing_extensions import List


class CacheManager:

    mesh_extensions: List[str] = [".obj", ".stl"]
    """
    The file extensions of mesh files.
    """

    def __init__(self, cache_dir: str, data_directory: List[str]):
        self.cache_dir = cache_dir
        # The directory where the cached files are stored.

        self.data_directory = data_directory
        # The directory where all resource files are stored.

    def update_cache_dir_with_object(self, path: str, ignore_cached_files: bool,
                                     object_description: 'ObjectDescription', object_name: str) -> str:
        """
        Checks if the file is already in the cache directory, if not it will be preprocessed and saved in the cache.
        """
        path_object = pathlib.Path(path)
        extension = path_object.suffix

        path = self.look_for_file_in_data_dir(path, path_object)

        self.create_cache_dir_if_not_exists()

        # save correct path in case the file is already in the cache directory
        cache_path = self.cache_dir + object_description.get_file_name(path_object, extension, object_name)

        # if file is not yet cached preprocess the description file and save it in the cache directory.
        if not self.is_cached(path, object_description) or ignore_cached_files:
            self.generate_description_and_write_to_cache(path, extension, cache_path, object_description)

        return cache_path

    def generate_description_and_write_to_cache(self, path: str, extension: str, cache_path: str,
                                                object_description: 'ObjectDescription') -> None:
        """
        Generates the description from the file at the given path and writes it to the cache directory.
        :param path: The path of the file to preprocess.
        :param extension: The file extension of the file to preprocess.
        :param cache_path: The path of the file in the cache directory.
        :param object_description: The object description of the file.
        """
        description_string = object_description.generate_description_from_file(path, extension)
        self.write_to_cache(description_string, cache_path)

    @staticmethod
    def write_to_cache(description_string: str, cache_path: str) -> None:
        """
        Writes the description string to the cache directory.
        :param description_string: The description string to write to the cache directory.
        :param cache_path: The path of the file in the cache directory.
        """
        with open(cache_path, "w") as file:
            file.write(description_string)

    def look_for_file_in_data_dir(self, path: str, path_object: pathlib.Path) -> str:
        """
        Looks for a file in the data directory of the World. If the file is not found in the data directory, this method
        raises a FileNotFoundError.
        :param path: The path of the file to look for.
        :param path_object: The pathlib object of the file to look for.
        """
        if re.match("[a-zA-Z_0-9].[a-zA-Z0-9]", path):
            for data_dir in self.data_directory:
                for file in os.listdir(data_dir):
                    if file == path:
                        return data_dir + f"/{path}"
                if path:
                    break

        if not path:
            raise FileNotFoundError(
                f"File {path_object.name} could not be found in the resource directory {self.data_directory}")

        return path

    def create_cache_dir_if_not_exists(self):
        """
        Creates the cache directory if it does not exist.
        """
        if not pathlib.Path(self.cache_dir).exists():
            os.mkdir(self.cache_dir)

    def is_cached(self, path: str, object_description: 'ObjectDescription') -> bool:
        """
        Checks if the file in the given path is already cached or if
        there is already a cached file with the given name, this is the case if a .stl, .obj file or a description from
        the parameter server is used.

        :param path: The path of the file to check.
        :param object_description: The object description of the file.
        :return: True if there already exists a cached file, False in any other case.
        """
        return True if self.check_with_extension(path) else self.check_without_extension(path, object_description)

    def check_with_extension(self, path: str) -> bool:
        """
        Checks if the file in the given ath exists in the cache directory including file extension.
        :param path: The path of the file to check.
        """
        file_name = pathlib.Path(path).name
        full_path = pathlib.Path(self.cache_dir + file_name)
        return full_path.exists()

    def check_without_extension(self, path: str, object_description: 'ObjectDescription') -> bool:
        """
        Checks if the file in the given path exists in the cache directory without file extension,
        the extension is added after the file name manually in this case.
        :param path: The path of the file to check.
        :param object_description: The object description of the file.
        """
        file_stem = pathlib.Path(path).stem
        full_path = pathlib.Path(self.cache_dir + file_stem + object_description.get_file_extension())
        return full_path.exists()
