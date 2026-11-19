import pandas as pd
import numpy as np
from pathlib import Path 

BASE_DIR = Path(__file__).resolve().parent

class MotionDatabase:

    def __init__(self, csv_file):

        self.df = pd.read_csv(BASE_DIR / csv_file)

        # entity database
        self.entities = {}

        self._parse_columns()

    # -------------------------------------------------
    # Parse CSV columns
    # rackl_x -> entity=rackl property=x
    # -------------------------------------------------
    def _parse_columns(self):

        for col in self.df.columns:

            split_idx = col.rfind('_')

            # skip invalid column
            if split_idx == -1:
                continue

            entity = col[:split_idx]
            prop = col[split_idx + 1:]

            if entity not in self.entities:
                self.entities[entity] = {}

            self.entities[entity][prop] = self.df[col].to_numpy()

    # -------------------------------------------------
    # Check if property exists
    # -------------------------------------------------
    def has_property(self, entity, prop):

        return (
            entity in self.entities and
            prop in self.entities[entity]
        )

    # -------------------------------------------------
    # Get property array
    # -------------------------------------------------
    def get(self, entity, prop):

        if not self.has_property(entity, prop):
            return None

        return self.entities[entity][prop]

    # -------------------------------------------------
    # Query relation between A and B
    # -------------------------------------------------

    def query_value(
        self,

        input_entity,
        input_property,

        input_value,

        output_entity,
        output_property
    ):

        # -------------------------
        # Get data
        # -------------------------

        x = self.get(input_entity, input_property)
        y = self.get(output_entity, output_property)

        # -------------------------
        # Check
        # -------------------------

        if x is None:

            raise ValueError(
                f'{input_entity}_{input_property} not found'
            )

        if y is None:

            raise ValueError(
                f'{output_entity}_{output_property} not found'
            )

        # -------------------------
        # Sort for interpolation
        # -------------------------

        sort_idx = np.argsort(x)

        x_sorted = x[sort_idx]
        y_sorted = y[sort_idx]

        # -------------------------
        # Interpolation
        # -------------------------

        output_value = np.interp(
            input_value,
            x_sorted,
            y_sorted
        )

        return output_value

if __name__ == "__main__":
    import matplotlib.pyplot as plt

    # =====================================================
    # Create searcher
    # =====================================================

    searcher = MotionDatabase('output_20260518_165406.csv')

    # =====================================================
    # Input sweep range
    # =====================================================

    input_values = np.linspace(
        np.min(searcher.get('rackl', 'v')),
        np.max(searcher.get('rackl', 'v')),
        200
    )

    # =====================================================
    # Query loop
    # =====================================================

    output_values = []

    for val in input_values:

        out = searcher.query_value(
            'rackl',
            'v',

            val,

            'tie_rodl',
            'v'
        )

        output_values.append(out)

        print(
            f"rackl_v = {val:.5f}  ->  tie_rodl_v = {out:.5f}"
        )

    output_values = np.array(output_values)

    # =====================================================
    # Plot
    # =====================================================

    plt.figure(figsize=(8,6))

    plt.plot(
        input_values,
        output_values
    )

    plt.xlabel('rackl_v')
    plt.ylabel('tie_rodl_v')

    plt.title('Motion Relationship')

    plt.grid(True)

    plt.show()