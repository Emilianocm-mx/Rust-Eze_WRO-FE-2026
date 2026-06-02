"""Carga de configuración desde YAML."""
import yaml

class Config:
    @staticmethod
    def load(path: str) -> "Config":
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        cfg = Config()
        cfg.__dict__.update(data)
        return cfg
