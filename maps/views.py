
from django.shortcuts import render
from django.http import JsonResponse

from .algorithms import find_path


# Display the homepage
def home(request):
    return render(request, 'maps/index.html')


# API for calculating the path
def find_path_api(request):

    if request.method != "GET":
        return JsonResponse(
            {"error": "Only GET requests are allowed"},
            status=405
        )

    source = request.GET.get("source", "A")
    destination = request.GET.get("destination", "J")
    algorithm = request.GET.get("algorithm", "bfs")

    try:

        result = find_path(
            source,
            destination,
            algorithm
        )

        return JsonResponse(result)

    except ValueError as error:

        return JsonResponse(
            {"error": str(error)},
            status=400
        )
