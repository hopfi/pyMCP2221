from setuptools import setup, find_packages

setup(
    name='pyMCP2221',
    version='0.0.1',
    packages=find_packages(include=['mcp2221', 'mcp2221.*'])
)