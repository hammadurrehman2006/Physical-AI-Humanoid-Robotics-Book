import os
import hashlib
from datetime import datetime
from typing import Dict, List
from pathlib import Path


class IncrementalUpdateService:
    """
    Service to track content changes and update only modified content
    """

    def __init__(self, content_path: str = "book/docs", tracking_file: str = ".content_tracking.json"):
        self.content_path = content_path
        self.tracking_file = tracking_file
        self.tracking_data = self._load_tracking_data()

    def _load_tracking_data(self) -> Dict[str, Dict[str, str]]:
        """
        Load content tracking data from file
        """
        import json
        tracking_file_path = Path(self.tracking_file)

        if tracking_file_path.exists():
            try:
                with open(tracking_file_path, 'r') as f:
                    return json.load(f)
            except:
                return {}
        return {}

    def _save_tracking_data(self):
        """
        Save content tracking data to file
        """
        import json
        tracking_file_path = Path(self.tracking_file)

        with open(tracking_file_path, 'w') as f:
            json.dump(self.tracking_data, f, indent=2)

    def _get_file_hash(self, file_path: str) -> str:
        """
        Calculate MD5 hash of a file
        """
        hash_md5 = hashlib.md5()
        with open(file_path, "rb") as f:
            # Read file in chunks to handle large files
            for chunk in iter(lambda: f.read(4096), b""):
                hash_md5.update(chunk)
        return hash_md5.hexdigest()

    def _get_modified_files(self) -> List[str]:
        """
        Get list of files that have been modified since last tracking
        """
        modified_files = []

        for root, dirs, files in os.walk(self.content_path):
            for file in files:
                if file.endswith(('.md', '.mdx')):
                    file_path = os.path.join(root, file)
                    current_hash = self._get_file_hash(file_path)

                    # Check if file is new or has been modified
                    if file_path not in self.tracking_data:
                        # New file
                        modified_files.append(file_path)
                        self.tracking_data[file_path] = {
                            "hash": current_hash,
                            "last_modified": datetime.now().isoformat()
                        }
                    else:
                        # Existing file - check if it's been modified
                        stored_hash = self.tracking_data[file_path]["hash"]
                        if stored_hash != current_hash:
                            modified_files.append(file_path)
                            self.tracking_data[file_path] = {
                                "hash": current_hash,
                                "last_modified": datetime.now().isoformat()
                            }

        # Check for deleted files
        files_in_path = set()
        for root, dirs, files in os.walk(self.content_path):
            for file in files:
                if file.endswith(('.md', '.mdx')):
                    files_in_path.add(os.path.join(root, file))

        deleted_files = []
        for tracked_file in list(self.tracking_data.keys()):
            if tracked_file not in files_in_path:
                deleted_files.append(tracked_file)
                del self.tracking_data[tracked_file]

        if deleted_files:
            print(f"Detected {len(deleted_files)} deleted files")

        return modified_files

    def get_update_plan(self) -> Dict[str, List[str]]:
        """
        Get plan for incremental updates (what to add/update, what to delete)
        """
        modified_files = self._get_modified_files()

        # For now, we're just tracking additions/updates and deletions
        # In a real implementation, you'd have more sophisticated tracking

        return {
            "to_update": modified_files,
            "to_delete": []  # Deletions are handled automatically in _get_modified_files
        }

    def execute_incremental_update(self, content_index_service) -> bool:
        """
        Execute incremental update based on tracked changes
        """
        try:
            update_plan = self.get_update_plan()

            # Process files to update
            for file_path in update_plan["to_update"]:
                print(f"Updating index for: {file_path}")
                success = content_index_service.update_file_index(file_path)
                if not success:
                    print(f"Failed to update {file_path}")
                    return False

            # Process files to delete
            for file_path in update_plan["to_delete"]:
                print(f"Deleting index for: {file_path}")
                success = content_index_service.delete_file_index(file_path)
                if not success:
                    print(f"Failed to delete {file_path}")
                    return False

            # Save tracking data
            self._save_tracking_data()

            print(f"Incremental update completed: {len(update_plan['to_update'])} files updated, {len(update_plan['to_delete'])} files deleted")
            return True
        except Exception as e:
            print(f"Error during incremental update: {str(e)}")
            return False

    def full_resync(self, content_index_service) -> bool:
        """
        Perform full resync of all content (for complete rebuild)
        """
        try:
            # Clear all tracking data
            self.tracking_data = {}

            # Index entire directory
            success = content_index_service.index_directory(self.content_path)

            if success:
                # Update tracking data with current state
                for root, dirs, files in os.walk(self.content_path):
                    for file in files:
                        if file.endswith(('.md', '.mdx')):
                            file_path = os.path.join(root, file)
                            current_hash = self._get_file_hash(file_path)
                            self.tracking_data[file_path] = {
                                "hash": current_hash,
                                "last_modified": datetime.now().isoformat()
                            }

                self._save_tracking_data()
                print("Full resync completed successfully")

            return success
        except Exception as e:
            print(f"Error during full resync: {str(e)}")
            return False