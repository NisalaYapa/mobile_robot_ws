import pkg_resources

required_packages = ['pyrvo2', 'Cython', 'casadi']

installed_packages = {pkg.key for pkg in pkg_resources.working_set}

for package in required_packages:
    if package in installed_packages:
        print(f"{package} is installed.")
    else:
        print(f"{package} is NOT installed.")

