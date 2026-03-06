
import math
import ast
import statistics
from typing import List, Tuple, Union

from langchain.agents import tool


@tool
def add_all(numbers: List[float]) -> float:
    """this is a math tool: Returns the sum of a list of numbers."""
    return sum(numbers)


@tool
def multiply_all(numbers: List[float]) -> float:
    """this is a math tool: Returns the product of a list of numbers."""
    result = 1
    for number in numbers:
        result *= number
    return result


@tool
def mean(numbers: List[float]) -> dict:
    """this is a math tool: Returns the mean of a list of numbers."""
    return {
        "mean": statistics.mean(numbers),
        "stdev": statistics.stdev(numbers),
    }


@tool
def median(numbers: List[float]) -> float:
    """this is a math tool: Returns the median of a list of numbers."""
    return statistics.median(numbers)


@tool
def mode(numbers: List[float]) -> List[float]:
    """this is a math tool: Returns the mode of a list of numbers."""
    return statistics.mode(numbers)


@tool
def variance(numbers: List[float]) -> float:
    """this is a math tool: Returns the variance of a list of numbers."""
    return statistics.variance(numbers)



@tool
def add(xy_pairs: List[Union[tuple, list, str]]) -> List[dict]:
    """
    this is a math tool:
    Performs addition on the input values x and y.

    :arg xy_pairs: A list containing tuples, lists, or string representations of tuples/lists
                   with two numbers (e.g. [(x1, y1), [x2, y2], "[(x3, y3)]", ...]).
    :return: A list of dictionaries where each dictionary has a key formatted as "x+y" and the corresponding sum.
             Invalid inputs are skipped and reported.
    """
    results = []
    errors = []

    for i, pair in enumerate(xy_pairs):
        try:
            # Try to parse the pair
            if isinstance(pair, (tuple, list)):
                parsed_pair = pair
            elif isinstance(pair, str):
                parsed_pair = ast.literal_eval(pair)
            else:
                raise ValueError(f"Unsupported type {type(pair)}")

            if not isinstance(parsed_pair, (tuple, list)) or len(parsed_pair) != 2:
                raise ValueError(f"Each element must contain exactly two numbers, got: {parsed_pair}")

            x, y = float(parsed_pair[0]), float(parsed_pair[1])
            results.append({f"{x}+{y}": x + y})
        except Exception as e:
            errors.append(f"Index {i}: {pair} → {str(e)}")

    # Optionally include error summary
    if errors:
        results.append({"errors": errors})

    return results
@tool
def subtract(xy_pairs: List[tuple]) -> List[dict]:
    """
    this is a math tool: 
    Performs subtraction on the input values x and y.

    :arg xy_pairs: A list of tuples containing the input values x and y (e.g. [(x1, y1), (x2, y2), ...])
    """
    results = []
    for pair in xy_pairs:
        # If pair is already a tuple or list, use it directly.
        if isinstance(pair, (tuple, list)):
            parsed_pair = pair
        elif isinstance(pair, str):
            # Try to safely evaluate the string to a tuple or list.
            try:
                parsed_pair = ast.literal_eval(pair)
            except Exception as e:
                raise ValueError(f"Invalid string format for pair: {pair}") from e
        else:
            raise ValueError(f"Unsupported type {type(pair)} for pair: {pair}")
        
        # Check that parsed_pair has exactly two elements.
        if not isinstance(parsed_pair, (tuple, list)) or len(parsed_pair) != 2:
            raise ValueError(f"Each element must contain exactly two numbers, got: {parsed_pair}")
        
        # Convert each value to float.
        try:
            x, y = float(parsed_pair[0]), float(parsed_pair[1])
        except Exception as e:
            raise ValueError(f"Cannot convert pair values to float: {parsed_pair}") from e

        result = {f"{x}-{y}": x - y}
        results.append(result)
    return results


@tool
def multiply(xy_pairs: List[tuple]) -> List[dict]:
    """
    this is a math tool: 
    Performs multiplication on the input values x and y.

    :arg xy_pairs: A list of tuples containing the input values x and y (e.g. [(x1, y1), (x2, y2), ...])
    """
    results = []
    for x, y in xy_pairs:
        result = {
            f"{x}*{y}": x * y,
        }
        results.append(result)
    return results


@tool
def divide(xy_pairs: List[tuple]) -> List[dict]:
    """
    this is a math tool: 
    Performs division on the input values x and y.

    :arg xy_pairs: A list of tuples containing the input values x and y (e.g. [(x1, y1), (x2, y2), ...])
    """
    results = []
    for x, y in xy_pairs:
        x,y = float(x), float(y)
        result = {
            f"{x}/{y}": x / y if y != 0 else "undefined",
        }
        results.append(result)
    return results


@tool
def exponentiate(xy_pairs: List[tuple]) -> List[dict]:
    """
    this is a math tool: 
    Performs exponentiation on the input values x and y.

    :arg xy_pairs: A list of tuples containing the input values x and y (e.g. [(x1, y1), (x2, y2), ...])
    """
    results = []
    for x, y in xy_pairs:
        try:
            x,y = float(x), float(y)
        except ValueError as e:
            return f"Invalid input for exponentiate: {e}"
        except Exception as e:
            return f"Unexpected error in exponentiate: {e}"
        
        result = {
            f"{x}^{y}": x**y,
        }
        results.append(result)
        
    return results


@tool
def modulo(xy_pairs: List[tuple]) -> List[dict]:
    """
    this is a math tool: 
    Performs modulo on the input values x and y.

    :arg xy_pairs: A list of tuples containing the input values x and y (e.g. [(x1, y1), (x2, y2), ...])
    """
    results = []
    for x, y in xy_pairs:
        x,y = float(x), float(y)
        result = {
            f"{x}%{y}": x % y if y != 0 else "undefined",
        }
        results.append(result)
    return results


@tool
def sine(x_values: List[float]) -> List[dict]:
    """
    this is a math tool: 
    Performs sine on the input values x.

    :arg x_values: A list of values x (e.g. [x1, x2, ...])
    """
    results = []
    for x in x_values:
        result = {
            f"sin({x})": math.sin(x),
        }
        results.append(result)
    return results


@tool
def cosine(x_values: List[float]) -> List[dict]:
    """
    this is a math tool: 
    Performs cosine on the input values x.

    :arg x_values: A list of values x (e.g. [x1, x2, ...])
    """
    results = []
    for x in x_values:
        result = {
            f"cos({x})": math.cos(x),
        }
        results.append(result)
    return results


@tool
def tangent(x_values: List[float]) -> List[dict]:
    """
    this is a math tool: 
    Performs tangent on the input values x.

    :arg x_values: A list of values x (e.g. [x1, x2, ...])
    """
    results = []
    for x in x_values:
        result = {
            f"tan({x})": math.tan(x),
        }
        results.append(result)
    return results


@tool
def asin(x_values: List[float]) -> List[dict]:
    """
    this is a math tool: 
    Performs arcsine on the input values x.

    :arg x_values: A list of values x (e.g. [x1, x2, ...])
    """
    results = []
    for x in x_values:
        try:
            result = {
                f"asin({x})": math.asin(x),
            }
        except ValueError:
            result = {
                f"asin({x})": "undefined",
            }
        results.append(result)
    return results


@tool
def acos(x_values: List[float]) -> List[dict]:
    """
    this is a math tool: 
    Performs arccosine on the input values x.

    :arg x_values: A list of values x (e.g. [x1, x2, ...])
    """
    results = []
    for x in x_values:
        try:
            result = {
                f"acos({x})": math.acos(x),
            }
        except ValueError:
            result = {
                f"acos({x})": "undefined",
            }
        results.append(result)
    return results


@tool
def atan(x_values: List[float]) -> List[dict]:
    """
    this is a math tool: 
    Performs arctangent on the input values x.

    :arg x_values: A list of values x (e.g. [x1, x2, ...])
    """
    results = []
    for x in x_values:
        result = {
            f"atan({x})": math.atan(x),
        }
        results.append(result)
    return results


@tool
def sinh(x_values: List[float]) -> List[dict]:
    """
    this is a math tool: 
    Performs hyperbolic sine on the input values x.

    :arg x_values: A list of values x (e.g. [x1, x2, ...])
    """
    results = []
    for x in x_values:
        result = {
            f"sinh({x})": math.sinh(x),
        }
        results.append(result)
    return results


@tool
def cosh(x_values: List[float]) -> List[dict]:
    """
    this is a math tool: 
    Performs hyperbolic cosine on the input values x.

    :arg x_values: A list of values x (e.g. [x1, x2, ...])
    """
    results = []
    for x in x_values:
        result = {
            f"cosh({x})": math.cosh(x),
        }
        results.append(result)
    return results


@tool
def tanh(x_values: List[float]) -> List[dict]:
    """
    this is a math tool: 
    Performs hyperbolic tangent on the input values x.

    :arg x_values: A list of values x (e.g. [x1, x2, ...])
    """
    results = []
    for x in x_values:
        result = {
            f"tanh({x})": math.tanh(x),
        }
        results.append(result)
    return results


@tool
def count_list(items: List) -> int:
    """
    this is a math tool: 
    Returns the number of items in a list."""
    return len(items)


@tool
def count_words(text: str) -> int:
    """
    this is a math tool: 
    Returns the number of words in a string."""
    return len(text.split())


@tool
def count_lines(text: str) -> int:
    """
    this is a math tool: 
    Returns the number of lines in a string."""
    return len(text.split("\n"))


@tool
def degrees_to_radians(degrees: List[float]):
    """
    this is a math tool: 
    Convert degrees to radians.

    :param degrees: A list of one or more degrees to convert to radians.
    """
    rads = {}
    for degree in degrees:
        rads[degree] = f"{degree * (3.14159 / 180)} radians."
    return rads


@tool
def radians_to_degrees(radians: List[float]):
    """
    this is a math tool: 
    Convert radians to degrees.

    :param radians: A list of one or more radians to convert to degrees.
    """
    degs = {}
    for radian in radians:
        degs[radian] = f"{radian * (180 / 3.14159)} degrees."
    return degs
