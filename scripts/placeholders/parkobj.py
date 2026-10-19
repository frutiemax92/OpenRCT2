"""Deterministic .parkobj (zip) writer shared by the object generators."""

import io
import zipfile

ZIP_DATE = (1980, 1, 1, 0, 0, 0)


def pack(files):
    """Deterministic ZIP: object.json first, then sorted paths, stored, fixed timestamps and attributes."""
    buffer = io.BytesIO()
    order = ["object.json"] + sorted(p for p in files if p != "object.json")
    with zipfile.ZipFile(buffer, "w", zipfile.ZIP_STORED) as z:
        for path in order:
            info = zipfile.ZipInfo(path, ZIP_DATE)
            info.compress_type = zipfile.ZIP_STORED
            info.external_attr = 0o644 << 16
            info.create_system = 3
            z.writestr(info, files[path])
    return buffer.getvalue()
