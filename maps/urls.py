
from django.urls import path
from . import views

urlpatterns = [
    path('', views.home, name='home'),

    path(
        'api/find-path/',
        views.find_path_api,
        name='find_path_api'
    ),
]
