"""
Device List Reader - Reads device list from Excel file.
"""

import pandas as pd
from typing import List, Optional, Union
import os


class DeviceListReader:
    """
    Class for reading device lists from Excel files.
    """

    def __init__(self):
        """Initialize the DeviceListReader."""
        pass

    def read_device_list(
        self,
        xlsx_filename: str,
        sheet_name: Optional[Union[str, int]] = None,
        column: int = 0,
        header: Optional[int] = None,
        skip_empty: bool = True
    ) -> List[str]:
        """
        Read device list from Excel file.

        Args:
            xlsx_filename: Path to the Excel file (.xlsx)
            sheet_name: Name or index of the sheet to read (default: first sheet)
            column: Column index to read (0-based, default: 0 for first column)
            header: Row index to use as header (default: None, no header)
            skip_empty: Whether to skip empty cells (default: True)

        Returns:
            List of device IDs/names as strings

        Raises:
            FileNotFoundError: If the Excel file doesn't exist
            ValueError: If column index is invalid

        Example:
            reader = DeviceListReader()
            devices = reader.read_device_list('devices.xlsx')
            print(devices)
        """
        if not os.path.exists(xlsx_filename):
            raise FileNotFoundError(f"Excel file not found: {xlsx_filename}")

        try:
            # Read Excel file
            df = pd.read_excel(
                xlsx_filename,
                sheet_name=sheet_name,
                header=header
            )

            if df.empty:
                return []

            # Validate column index
            if column < 0 or column >= len(df.columns):
                raise ValueError(f"Column index {column} is out of range. "
                               f"File has {len(df.columns)} columns.")

            # Extract device list from specified column
            device_column = df.iloc[:, column]

            # Convert to list and filter
            device_list = device_column.tolist()

            # Convert to strings and filter
            result = []
            for item in device_list:
                # Skip NaN/null values
                if pd.isna(item):
                    if not skip_empty:
                        result.append('')
                    continue

                # Convert to string and strip whitespace
                device_str = str(item).strip()

                # Skip empty strings if requested
                if skip_empty and not device_str:
                    continue

                result.append(device_str)

            return result

        except Exception as e:
            raise ValueError(f"Error reading Excel file: {str(e)}")


def read_device_list(
    xlsx_filename: str,
    sheet_name: Optional[Union[str, int]] = None,
    column: int = 0,
    header: Optional[int] = None,
    skip_empty: bool = True
) -> List[str]:
    """
    Convenience function to read device list from Excel file.
    Reads from the specified column of the specified sheet.

    Args:
        xlsx_filename: Path to the Excel file (.xlsx)
        sheet_name: Name or index of the sheet to read (default: first sheet)
        column: Column index to read (0-based, default: 0 for first column)
        header: Row index to use as header (default: None, no header)
        skip_empty: Whether to skip empty cells (default: True)

    Returns:
        List of device IDs/names as strings

    Example:
        devices = read_device_list('devices.xlsx', sheet_name='Sheet1')
        print(devices)
    """
    reader = DeviceListReader()
    return reader.read_device_list(xlsx_filename, sheet_name=sheet_name, column=column, header=header, skip_empty=skip_empty)
