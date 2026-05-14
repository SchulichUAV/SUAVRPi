import os
from dataclasses import dataclass


@dataclass(frozen=True)
class CaptureFiles:
    number: int
    stem: str
    image_path: str
    metadata_path: str


class CaptureFileSequence:
    """Keeps image + metadata filenames moving forward across restarts."""

    def __init__(self, image_dir, padding=5, counter_file=".last_image_number"):
        self.image_dir = image_dir
        self.padding = padding
        self.counter_path = os.path.join(image_dir, counter_file)

    def files_for(self, number):
        stem = f"{number:0{self.padding}d}"
        return CaptureFiles(
            number=number,
            stem=stem,
            image_path=os.path.join(self.image_dir, f"{stem}.jpg"),
            metadata_path=os.path.join(self.image_dir, f"{stem}.json"),
        )

    def last_saved_number(self, client_image_count=0):
        return max(
            0,
            client_image_count,
            self._read_counter(),
            self._latest_number_in_image_dir(),
        )

    def remember_saved(self, number):
        try:
            os.makedirs(self.image_dir, exist_ok=True)
            temp_counter_path = f"{self.counter_path}.tmp"
            with open(temp_counter_path, "w") as f:
                f.write(f"{number}\n")
            os.replace(temp_counter_path, self.counter_path)
        except OSError as e:
            print(f"WARN: Could not update image counter file: {e}")

    def _read_counter(self):
        try:
            with open(self.counter_path, "r") as f:
                return max(0, int(f.read().strip() or "0"))
        except FileNotFoundError:
            return 0
        except (OSError, ValueError) as e:
            print(f"WARN: Could not read image counter file: {e}")
            return 0

    def _latest_number_in_image_dir(self):
        try:
            file_names = os.listdir(self.image_dir)
        except FileNotFoundError:
            return 0
        except OSError as e:
            print(f"WARN: Could not scan image directory for existing files: {e}")
            return 0

        latest_number = 0
        for file_name in file_names:
            number = self._number_from_file_name(file_name)
            if number is not None:
                latest_number = max(latest_number, number)
        return latest_number

    def _number_from_file_name(self, file_name):
        stem, extension = os.path.splitext(file_name)
        if extension.lower() not in {".jpg", ".jpeg", ".png", ".json"}:
            return None

        if stem.isdigit():
            return int(stem)

        return None
