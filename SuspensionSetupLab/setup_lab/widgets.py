"""Shared widgets for the project and analysis forms."""

from tkinter import ttk


def option(parent, choices, variable, width=None):
    return ttk.Combobox(parent, values=tuple(choices.values()), textvariable=variable, state="readonly", width=width)


def key_for(choices, label):
    return next(key for key, value in choices.items() if value == label)


def entry(parent, row, column, label, variable, width=12):
    ttk.Label(parent, text=label).grid(row=row, column=column, sticky="w", padx=(0, 8), pady=4)
    field = ttk.Entry(parent, textvariable=variable, width=width)
    field.grid(row=row, column=column + 1, sticky="ew", padx=(0, 14), pady=4)
    return field


